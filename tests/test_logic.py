def test_haptic_decay_logic():
    class Obstacle:
        def __init__(self, active, persistence):
            self.active = active
            self.persistence = persistence

    obstacles = [Obstacle(True, 2)] + [Obstacle(False, 0) for _ in range(9)]
    decay_tick = 0

    def planner_haptic_decay_tick():
        nonlocal decay_tick
        decay_tick += 1
        if decay_tick >= 100:
            decay_tick = 0
            for obs in obstacles:
                if obs.active:
                    obs.persistence -= 1
                    if obs.persistence <= 0:
                        obs.active = False

    for _ in range(99):
        planner_haptic_decay_tick()
    assert obstacles[0].persistence == 2

    planner_haptic_decay_tick()
    assert obstacles[0].persistence == 1

    for _ in range(100):
        planner_haptic_decay_tick()
    assert obstacles[0].persistence == 0
    assert not obstacles[0].active


def test_variable_compliance_logic():
    g_lsm_stress_level = 0.8
    norm_torque = 0.5

    compliance_factor = 1.0 - (g_lsm_stress_level * 0.8)
    if compliance_factor < 0.1:
        compliance_factor = 0.1

    assert abs(compliance_factor - 0.36) < 1e-5

    commanded_torque = ((norm_torque + 1.0) / 2.0) * 1000.0 * compliance_factor
    assert abs(commanded_torque - (1.5 / 2.0) * 1000.0 * 0.36) < 1e-5


def test_snn_pruning_logic():
    class SNN:
        def __init__(self):
            self.w_out = [[0.5, 0.005, -0.008, -0.1]]
            self.w_rec = [[0.8, 0.002, -0.009, -0.5]]

        def prune(self, threshold):
            for i in range(len(self.w_out)):
                for j in range(len(self.w_out[0])):
                    if abs(self.w_out[i][j]) < threshold:
                        self.w_out[i][j] = 0.0
            for i in range(len(self.w_rec)):
                for j in range(len(self.w_rec[0])):
                    if abs(self.w_rec[i][j]) < threshold:
                        self.w_rec[i][j] = 0.0

    snn = SNN()
    snn.prune(0.01)

    assert snn.w_out == [[0.5, 0.0, 0.0, -0.1]]
    assert snn.w_rec == [[0.8, 0.0, 0.0, -0.5]]


def test_ab_shadow_banking_and_plasticity_mask_logic():
    class WeightBank:
        def __init__(self, n_out=4, n_res=64):
            self.w_out = [[0.1 for _ in range(n_res)] for _ in range(n_out)]
            self.eta_out = [[1.0 for _ in range(n_res)] for _ in range(n_out)]

    class LSMDoubleBuffer:
        def __init__(self):
            self.banks = [WeightBank(), WeightBank()]
            self.active_bank = 0
            self.ab_eval_active = False

        def prepare_shadow(self):
            shadow = self.active_bank ^ 1
            for i in range(4):
                for j in range(64):
                    self.banks[shadow].w_out[i][j] = self.banks[self.active_bank].w_out[i][j]
                    self.banks[shadow].eta_out[i][j] = self.banks[self.active_bank].eta_out[i][j]
            return shadow

        def patch_synapse(self, post_idx, pre_idx, new_w, new_eta):
            shadow = self.prepare_shadow()
            self.banks[shadow].w_out[post_idx][pre_idx] = new_w
            self.banks[shadow].eta_out[post_idx][pre_idx] = new_eta
            self.active_bank ^= 1
            self.ab_eval_active = True

        def commit(self):
            inactive = self.active_bank ^ 1
            for i in range(4):
                for j in range(64):
                    self.banks[inactive].w_out[i][j] = self.banks[self.active_bank].w_out[i][j]
                    self.banks[inactive].eta_out[i][j] = self.banks[self.active_bank].eta_out[i][j]
            self.ab_eval_active = False

        def rollback(self):
            self.active_bank ^= 1
            self.ab_eval_active = False

    lsm = LSMDoubleBuffer()
    assert lsm.active_bank == 0
    assert lsm.banks[0].w_out[0][5] == 0.1

    # Stage patch: freeze synapse (eta = 0.0) with new weight 0.85
    lsm.patch_synapse(0, 5, 0.85, 0.0)
    assert lsm.active_bank == 1
    assert lsm.banks[1].w_out[0][5] == 0.85
    assert lsm.banks[1].eta_out[0][5] == 0.0
    # Original baseline in bank 0 remains untouched during shadow eval
    assert lsm.banks[0].w_out[0][5] == 0.1
    assert lsm.banks[0].eta_out[0][5] == 1.0

    # Test rollback
    lsm.rollback()
    assert lsm.active_bank == 0
    assert lsm.banks[lsm.active_bank].w_out[0][5] == 0.1

    # Test patch and commit
    lsm.patch_synapse(1, 10, -0.40, 2.5)
    assert lsm.active_bank == 1
    lsm.commit()
    assert lsm.active_bank == 1
    assert not lsm.ab_eval_active
    assert lsm.banks[0].w_out[1][10] == -0.40
    assert lsm.banks[1].w_out[1][10] == -0.40


def test_emergency_rollback_logic():
    MAX_SAFE_ACCEL_JERK = 2.5
    active_bank = 1
    ab_eval_active = True
    stress_high_ticks = 0

    def check_emergency_rollback(current_jerk, stress_level):
        nonlocal active_bank, ab_eval_active, stress_high_ticks
        if not ab_eval_active:
            return False
        jerk_exceeded = (current_jerk > MAX_SAFE_ACCEL_JERK)
        if stress_level > 0.85:
            stress_high_ticks += 1
        else:
            stress_high_ticks = 0
        stress_exceeded = (stress_high_ticks >= 3)
        if jerk_exceeded or stress_exceeded:
            active_bank ^= 1
            ab_eval_active = False
            return True
        return False

    # Safe step
    assert not check_emergency_rollback(0.8, 0.4)
    assert active_bank == 1
    assert ab_eval_active

    # Jerk spike -> immediate rollback
    assert check_emergency_rollback(3.2, 0.4)
    assert active_bank == 0
    assert not ab_eval_active

    # Test stress high ticks trigger
    active_bank = 1
    ab_eval_active = True
    stress_high_ticks = 0
    assert not check_emergency_rollback(0.2, 0.90)  # tick 1
    assert not check_emergency_rollback(0.2, 0.90)  # tick 2
    assert check_emergency_rollback(0.2, 0.90)      # tick 3 -> rollback!
    assert active_bank == 0
    assert not ab_eval_active


def test_predictive_coding_bridge_and_swarm_synthesis_logic():
    # 1. Predictive coding hook simulation
    hidden_weights = [[0.0 for _ in range(60)] for _ in range(16)]

    def patch_layer(layer, post, pre, w):
        if layer == "pred_coding" and post < 16 and pre < 60:
            hidden_weights[post][pre] = w
            return True
        return False

    assert patch_layer("pred_coding", 2, 5, 0.42)
    assert hidden_weights[2][5] == 0.42

    # 2. Swarm broadcast logic
    broadcast_packet = None

    def broadcast_patch(post_idx, pre_idx, new_w, new_eta):
        nonlocal broadcast_packet
        broadcast_packet = {
            "type": "MSG_TYPE_WEIGHT_PATCH",
            "post_idx": post_idx,
            "pre_idx": pre_idx,
            "w": new_w,
            "eta": new_eta
        }

    broadcast_patch(1, 25, 0.75, 1.0)
    assert broadcast_packet["type"] == "MSG_TYPE_WEIGHT_PATCH"
    assert broadcast_packet["w"] == 0.75


def test_dreaming_consolidation_and_surface_profiling_logic():
    # 1. Dreaming consolidation buffer
    dream_buffer = []

    def record_memory(inp, target, diss, curr):
        if diss > 0.2:
            dream_buffer.append({"input": inp, "target": target, "diss": diss})

    record_memory([0.1]*10, [0.2]*5, 0.15, 0.5) # filtered out
    assert len(dream_buffer) == 0

    record_memory([0.1]*10, [0.2]*5, 0.45, 0.5) # accepted
    assert len(dream_buffer) == 1
    assert dream_buffer[0]["diss"] == 0.45

    # 2. Haptic surface micro-vibration stiffness metric
    variance = 0.002
    stiffness = 1.0 / (1.0 + 50.0 * variance)
    assert abs(stiffness - 1.0 / 1.1) < 1e-4


def test_som_multimodal_fusion_and_optical_flow_slip_logic():
    # 1. SOM BMU selection logic
    weights = [
        [[0.1, 0.2], [0.8, 0.9]],
        [[0.3, 0.4], [0.5, 0.6]]
    ]
    sample = [0.75, 0.85]
    best_x, best_y, min_dist = 0, 0, 1e9
    for x in range(2):
        for y in range(2):
            d = (sample[0] - weights[x][y][0])**2 + (sample[1] - weights[x][y][1])**2
            if d < min_dist:
                min_dist = d
                best_x, best_y = x, y
    assert best_x == 0 and best_y == 1

    # 2. Optical flow slip detection logic
    expected_vx, expected_vy = 0.2, 0.0
    measured_flow_dx, measured_flow_dy = 0.02, 0.0
    squal = 120

    diff_x = expected_vx - measured_flow_dx
    diff_y = expected_vy - measured_flow_dy
    discrepancy = (diff_x**2 + diff_y**2)**0.5
    slip_detected = discrepancy > 0.15 and squal > 50

    assert slip_detected
    assert discrepancy > 0.17
