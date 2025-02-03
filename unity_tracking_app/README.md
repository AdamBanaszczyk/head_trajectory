This is an application that we developed to log _CenterEyeAnchor_ data which corresponds to the pose of the VR headset in the Unity environment. It also logs hand pose data with _LeftHandAnchor_ and _RightHandAnchor_.  

It is compatible with Quest headsets since obtaining such data required using specific Oculus assets together with _Oculus Interactions SDK_. The exact distribution of the objects used for this task can be seen below.
<p align="center">
  <img src="https://github.com/user-attachments/assets/68cc19e5-7913-4827-a3d9-8674ff1902ad" width="800" height="auto">
</p>  

However, the most important part of this application, enabling data logging, was the creation of specialized scripts directly responsible for reading and writing data, placed in the Assets folder. We provide them for use and encourage their adoption in similar applications, as other headsets will require a similar solution.

In-app surrounding objects aren’t crucial to the main functionality. They were added to prevent an empty environment and ensure the headset renders something, as it would in any other use case. The app itself also served as a playground for practicing Unity development.

