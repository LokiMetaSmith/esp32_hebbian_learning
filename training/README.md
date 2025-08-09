# Training

This directory contains files related to training the neural network.

## Files

*   `kinematic_model.py`: A script that defines the kinematic model of the robotic arm.
*   `mcp_server.py`: A Python implementation of the MCP server, used for training and testing.
*   `njf_hebbian_trainer.py`: A script for training the neural network using a Neuro-Fuzzy Hebbian learning rule.
*   `rockpool_trainer.py`: A script for training the neural network using the Rockpool library.
*   `snn_kinematic_model.py`: A script that defines the kinematic model of the robotic arm for the SNN.
*   `snn_trainer.py`: A script for training the Spiking Neural Network (SNN).

## Dependencies

The training scripts require several Python libraries, including `torch`, `snntorch`, and `numpy`. All dependencies are listed in the `requirements.txt` file in the root of the repository.

Install them using pip:
```bash
# from the root of the repository
pip install -r requirements.txt
```

## Running the Training Scripts

To run the training scripts, execute them from the root of the repository. For example:

```bash
python training/njf_hebbian_trainer.py
```
