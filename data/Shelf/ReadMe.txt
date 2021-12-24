CAMP - TUM
contact person: Vasilis Belagiannis (belagian@in.tum.de)
-------------------

Shelf dataset

Body Joint 2D and 3D Ground-Truth (GT) for 4 Humans (5 camera views)

GT Structure
-------------------

File: actorsGT.mat
	-actor2D (2D ground-truth for each individual)
	-actor3D (3D ground-truth for each individual)

Run the scripts "visualize2DGT.m" and "visualize3DGT.m" in order to check the 2D/3D joint GT in different views and for different humans.

The joint indices are defined as:
01	Right Ankle
02	Right Knee
03	Right Hip
04	Left Hip
05	Left Knee
06	Left Ankle
07	Right Wrist
08	Right Elbow
09	Right Shoulder
10	Left Shoulder
11	Left Elbow
12	Left Wrist
13	Bottom Head
14	Top Head

Calibration
------------------

Projection Matrices:

P0.txt
P1.txt
P2.txt
P3.txt
P4.txt

OR

prjectionMat.mat

In order to get RT or K matrices, execute the script "producePmat.m"

Testing
-------

Frames:

	- [300 - 600]
	
Citation
--------

@inproceedings {belagiannis20143d,
	title = {3D Pictorial Structures for Multiple Human Pose Estimation},
	booktitle={CVPR 2014-IEEE International Conference on Computer Vision and Pattern Recognition},
	year = {2014},
	month = {June},
	author = {Belagiannis, Vasileios and Amin, Sikandar and Andriluka, Mykhaylo and Schiele, Bernt and Navab, Nassir and Ilic, Slobodan}
	organization={IEEE}
}