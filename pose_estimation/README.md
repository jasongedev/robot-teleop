# 3D Pose Estimation Model

The pre-trained model and most of the inference code in this folder came from the [Open Model Zoo](https://github.com/openvinotoolkit/open_model_zoo) repository, which in turn was based on the [Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) and [Single-Shot Multi-Person 3D Pose Estimation From Monocular RGB](https://arxiv.org/pdf/1712.03453.pdf) papers and trained on the CMU Panoptic dataset.

I retained only the portions neccesary for GPU inference. Most of my original work was performed in the JointAngleCalculator class.

### joint_angle_calculator

This class calculates the extension and rotation angles of each limb based on inferred joint skeleton positions in 3D space, following the application of Euler's rotation theorem.
