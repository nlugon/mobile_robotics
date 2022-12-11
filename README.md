
## Abstract
This final project represents a culmination of all of the topics covered during MICRO-452. The goal was to place a Thymio robot within an obstacle field, and use computer vision, path planning, global navigation, and local navigation to direct it to a goal. The first step of the process was to decide on the environment the Thymio would be placed within. We considered the examples provided from previous semesters, and eventually we decided on a simple white background with 3D colored blocks as obstacles.</br> The intention was to have enough contrast between the obstacles and the background to allow an accurate contour detection. </br>The goal and the Thymio were both identified using ArUco marker detection. This allowed for an accurate tracking of both the position and orientation of the Thymio, which were integral in applying a global navigation framework based on computer vision. We were able in this project to successfully perfom the global navigation **with and without** camera using the filtering, **avoid** the **obstacles** locally using sensor data and to do a **kidnapping** test.

## Demo


## Implementation
For the implementation of this project and a detailed description of each method used, please refer to the [notebook](MobileRobotics_Final_Report_Run.ipynb)

## timeline

| Date | Goal |
|-|-|
| 15/11/2022 | First meeting and setting brief goals |
| 18/11/2022 | Expressing area of interest of each member and doing research |
| 21/11/2022 | Setting Milestones for each methods and agreed on a structure |
| 27/11/2022 | Working modules with tests |
| 4/12/2022 | Optimizing our parts and supporting each others |
| 7/12/2022 | Putting everything together |
| 9/12/2022 | Performing final demo |
| 12/12/2022 | **Presetation day** |


## Organization within the team
Each member of the team had expressed his area of interest. We had each member concentrating on a specific area as well as having an overview over the other remaining methods. We end up each doing a part and contributing in the other parts. 

## Sources

**Main reference**: Course material of Basics of Mobile Robotics </br> </br>
Zhu, X., Lyu, S., Wang, X., & Zhao, Q. (2021). TPH-YOLOv5: Improved YOLOv5 Based on Transformer Prediction Head for Object Detection on Drone-captured Scenarios. arXiv. https://doi.org/10.48550/ARXIV.2108.11539 </br>
</br>
Sampathkrishna, A. (2022). ArUco Maker based localization and Node graph approach to mapping. arXiv. https://doi.org/10.48550/ARXIV.2208.09355

---

## Setting up the project 

### Project Dependencies 
After activating your virtualenv in the root directory of the project install the necessary Dependencies by running
```bat
pip install -r requirements.txt
```
The requirement list will be updated and added along the project to make a reproducible code 
