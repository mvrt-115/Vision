import network

class Pose:
    """
    Class to get calculate and update pose location from the fudicials
    """

    def __init__(self, tagLocations):
        """
        Paramters
        ---------
        tagLocations: a dictonary of the tag ids and their absolute locations on the field
            {tagId: [x, y, theta],
            ...}

            theta: degrees you need to turn clockwise such that the back of the tag faces 
                the direction of y = 0 and is parallel with the x axis2   
        """

        self.tagLocs = tagLocations

    def update_pose(self, tagId, relX, relY, relTheta):
        """
        Calculates the absolute pose from the relative pose and updates the network table
        values: 'fe x', 'fe y', and 'fe theta'

        Paramters
        ---------
        tagId: april tag fudicial id
        rel[x, y, theta]: pose of the camera relative to the tag where the top left is (0, 0)
            and x increases to the right, and y increases down
        """

        tagLoc = self.tagLocs[tagId]

        absX = tagLoc[0] + relX
        absY = tagLoc[1] + relY
        
        network.updatePose(absX, absY, relTheta)