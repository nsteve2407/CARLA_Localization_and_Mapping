import open3d
import pypcd

class ScanMatch :
    def __init__(self):
        self.map = None #Load HD Map
        self.ndt = None

    
    def scan_match_prev_scan(self,source,target,initial_guess):
        #Inputs:
        #Soucre: the pointcloud from the current time step
        # Target: PointCloud from previous time step

        # Output:
        # pose estimate in the form [x,y,theta]


        return 0
    
    def scan_match_map(self,source,initial_guess):
        #Inputs:
        #Soucre: the pointcloud from the current time step
        # Target: self.map cloud

        # Output:
        # pose estimate in the form [x,y,theta]


        return 0

