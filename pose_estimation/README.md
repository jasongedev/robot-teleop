# 3D Pose Estimation Model

The pre-trained model and most of the inference code in this folder came from the [Open Model Zoo](https://github.com/openvinotoolkit/open_model_zoo) repository, which in turn was based on the [Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) and [Single-Shot Multi-Person 3D Pose Estimation From Monocular RGB](https://arxiv.org/pdf/1712.03453.pdf) papers and trained on the CMU Panoptic dataset.

I retained only the portions neccesary for GPU inference. Most of my original work was performed in the joint_angle_calculator class.

### joint_angle_calculator

This class calculates the extension (Φ) and rotation (ψ) angles of each limb based on inferred skeleton joint positions in 3D space by applying Euler's rotation theorem.
