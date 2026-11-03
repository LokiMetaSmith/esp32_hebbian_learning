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
