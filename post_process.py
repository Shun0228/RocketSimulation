import os
import math
import numpy as np
import quaternion as qt
import environment as env
import numpy.linalg as nplin
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
from mpl_toolkits.mplot3d import Axes3D


class JudgeInside():
    def __init__(self):
        print("Jude inside : ON")


    def set_limit_area(self, xy_range):

        # Check range area is close or not
        if np.allclose(xy_range[0,:], xy_range[-1,:]):
            print("")
            print("Range is close.")
            print("")

            self.xy_range = xy_range

        else:
            print("")
            print("Range area is not close.")
            print("Connect first point and last point automatically.")
            print("")

            point_first = xy_range[0,:]
            self.xy_range = np.vstack((xy_range, point_first))


    def set_limit_circle(self, xy_center, lim_radius = 50.0):

        self.xy_center = xy_center
        self.lim_radius = lim_radius


    def judge_inside(self, check_point):

        # Check limit circle area is defined
        try:
            self.xy_center
            circle_flag = True

        except AttributeError:
            circle_flag = False


        # Initialize count of line cross number
        cross_num = 0

        # Count number of range area
        point_num = self.xy_range.shape[0]

        # Judge inside or outside by cross number
        for point in range(point_num - 1):

            point_ymin = np.min(self.xy_range[point:point+2, 1])
            point_ymax = np.max(self.xy_range[point:point+2, 1])

            if check_point[1] == self.xy_range[point, 1]:

                if check_point[0] < self.xy_range[point, 0]:
                    cross_num += 1

                else:
                    pass

            elif point_ymin < check_point[1] < point_ymax:

                dx = self.xy_range[point+1, 0] - self.xy_range[point, 0]
                dy = self.xy_range[point+1, 1] - self.xy_range[point, 1]

                if dx == 0.0:
                    # Line is parallel to y-axis
                    judge_flag = self.xy_range[point, 1] - check_point[1]

                elif dy == 0.0:
                    # Line is parallel to x-axis
                    judge_flag = -1.0

                else:
                    # y = ax + b (a:slope,  b:y=intercept)
                    slope = dy / dx
                    y_intercept = self.xy_range[point, 1] - slope * self.xy_range[point, 0] 

                    # left:y,  right:ax+b
                    left_eq = check_point[1]
                    right_eq = slope * check_point[0] + y_intercept

                    judge_flag = slope * (left_eq - right_eq)


                if judge_flag > 0.0:
                    # point places left side of line 
                    cross_num += 1.0

                elif judge_flag < 0.0:
                    # point places right side of line
                    pass

            else:
                pass

        # odd number : inside,  even number : outside
        judge_result = np.mod(cross_num, 2)
        
        # check judge circle mode
        if circle_flag == True:

            center_num = self.xy_center.shape[0]

            for center in range(center_num):
                # Compute distance between drop_point and center of limit circle
                length_point = np.sqrt((check_point[0] - self.xy_center[center, 0])**2 + \
                                       (check_point[1] - self.xy_center[center, 1])**2)

                # Judge in limit circle or not
                if length_point <= self.lim_radius:
                    judge_result = np.bool(False)

                else:
                    pass

        else:
            pass

        # Convert from float to bool (True:inside,  False:outside)
        judge_result = np.bool(judge_result)

        return judge_result


class PostProcess():
    """
    Compute physical values in this method.

    input:
        result : rocket dynamics output
        wind_cond : wind condition (0:veloity, 1:direction)

    """

    def __init__(self):
        """
        Compute physical values in this method.

        input:
            result : rocket dynamics output
            wind_cond : wind condition (0:veloity, 1:direction)

        """
        

        # Define constant value
        self.gas_R = 287
        self.gas_gamma = 1.4 

        # Initial case number
        self.case = -1

        # Generate empty matrix to store the data
        max_case = 1000

        self.wind_cond = np.zeros([max_case, 2])
        self.drop_point = np.zeros([max_case, 2])
        self.max_height = np.zeros(max_case)
        self.max_vel = np.zeros(max_case)
        self.max_mach = np.zeros(max_case)
        self.launch_clear_time = np.zeros(max_case)
        self.launch_clear_vel = np.zeros(max_case)


    def set_variety(self, result ,wind_cond):

        #print("post processing...")

        # Set total step number
        total_step = result.shape[0]

        # Count the number of cases
        self.case += 1 
        #print("Case number : {0}".format(self.case))

        # Generate matrix
        self.wind = np.zeros([total_step, 3])
        self.angle_body_rad = np.zeros([total_step, 2])
        self.dcm_body = np.zeros([3, 3, total_step])
        self.vel_body = np.zeros([total_step, 3])
        
        # Allocate value from result
        self.mass = result[:,0]
        self.len_CG = result[:,1]
        self.Iyz = result[:,2]
        self.Ix = result[:,3]
        self.pos = result[:,4:7]
        self.height = result[:,6]
        self.drop_point[self.case,:] = self.pos[-1,0:2]
        self.vel = result[:,7:10]
        self.omega = result[:,10:13]
        self.quat = result[:,13:17]
        self.time_vec = result[:,-1]
        self.dt = self.time_vec[1] - self.time_vec[0]

        # Compute vector length
        self.pos_norm = nplin.norm(self.pos, axis=1)
        self.vel_norm = nplin.norm(self.vel, axis=1)

        # Compute height related parameter
        for step in range(total_step):

            # Gas parameter
            self.temp, self.pres, self.dens = env.gas_param(self.height[step])

            # Wind parameter
            self.wind[step,:] = env.wind_method(self.height[step], wind_cond)
            self.dcm_body[:,:,step] = qt.quat2dcm(self.quat[step,:])
            self.vel_body[step,:] = self.dcm_body[:,:,step] @ (self.vel[step,:] - self.wind[step,:])
            
            # Launch clear time/velocity
            if self.pos_norm[step] <= 5.0:
                self.launch_clear_time[self.case] = step * self.dt
                self.launch_clear_vel[self.case] = self.vel_norm[step]

                # Body angle constraint
                self.angle_body_rad[step,:] = 0.0

            else:
                # Compute body angle
                self.angle_body_rad[step,0] = np.arctan(self.vel_body[step,2]/self.vel_body[step,0])
                self.angle_body_rad[step,1] = np.arctan(self.vel_body[step,1]/self.vel_body[step,0])

        # 
        self.sonic_speed = np.sqrt(self.gas_gamma * self.gas_R * self.temp)
        self.vel_mach = self.vel_norm / self.sonic_speed
        self.angle_body_deg = np.rad2deg(self.angle_body_rad)

        # Store wind condition
        self.wind_cond[self.case, :] = wind_cond

        # Compute max number
        self.max_height[self.case] = np.max(self.height)
        self.max_vel[self.case] = np.max(self.vel_norm)
        self.max_mach[self.case] = np.max(self.vel_mach)
        

    def plot_detail(self):
        """ Plot result in detail """

        # Show results
        print("Launch clear time     : {0} sec".format(self.launch_clear_time[0]))
        print("Launch clear velocity : {0} m/s".format(self.launch_clear_vel[0]))
        print("")
        print("max height   : {0} m".format(self.max_height[0]))
        print("max velocity : {0} m/s".format(self.max_vel[0]))
        print("max Mach     : {0}".format(self.max_mach[0]))
        print("drop point   : {0}".format(self.drop_point[0,:]))

       # plt.figure()
       # plt.plot(self.time_vec, self.height, label='height')
       ## plt.plot(self.time_vec, self.angle_body_deg[:,1], label='beta')
       # plt.xlabel("time[sec]")
       # plt.ylabel("Z[m]")
       # plt.legend()

        plt.show()

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.plot(self.pos[:,0], self.pos[:,1], self.pos[:,2])

        range_lim = np.max(np.absolute(self.pos))
        ax.set_xlim(-range_lim,range_lim)
        ax.set_ylim(-range_lim,range_lim)
        ax.set_zlim(0,)

        ax.set_xlabel("X[m]")
        ax.set_ylabel("Y[m]")
        ax.set_zlabel("Up[m]")

        plt.show()


    def set_map(self, place):
        """ Set lat/lon coordinates to define MAP """

        earth_radius = 6378150.0    # [km]

        if place == 'Izu_land':

            # Set lat/long coordinates
            # point_origin : map origin
            # point_center : circle limt area
            # point_range  : limit area vertex
            self.lim_radius = 50.0   # define circle limit area

            self.point_origin = np.array([34.735972, 139.420944])

            self.point_center = np.array([[34.735972, 139.420944],
                                          [34.735390, 139.421377],
                                          [34.731230, 139.423150]])

            self.point_range = np.array([[34.735715, 139.420922],
                                         [34.731750, 139.421719],
                                         [34.733287, 139.424590],
                                         [34.736955, 139.426038],
                                         [34.738908, 139.423597],
                                         [34.740638, 139.420681],
                                         [34.741672, 139.417387],
                                         [34.735715, 139.420922],
                                         ])

            self.point_center_rel = self.point_center - self.point_origin
            self.point_range_rel = self.point_range - self.point_origin

            # Set magnetic declination
            self.mag_dec_deg = -7.53   # [deg]

            mag_dec_rad = np.deg2rad(self.mag_dec_deg)
            mat_rot = np.array([[np.cos(mag_dec_rad), -1 * np.sin(mag_dec_rad)],
                                [np.sin(mag_dec_rad), np.cos(mag_dec_rad)]])

            # Convert lat/lon to meter
            self.lat2met = 2 * math.pi * earth_radius / 360.0
            self.lon2met = 2 * math.pi * earth_radius * np.cos(np.deg2rad(self.point_origin[0])) / 360.0
            
            # Convert from lat/long to meter (ENU coordinate)
            self.xy_center = np.zeros(self.point_center.shape)
            self.xy_range = np.zeros(self.point_range.shape)

            self.xy_center[:,0] = self.lon2met * self.point_center_rel[:,1]
            self.xy_center[:,1] = self.lat2met * self.point_center_rel[:,0]
            self.xy_range[:,0] = self.lon2met * self.point_range_rel[:,1]
            self.xy_range[:,1] = self.lat2met * self.point_range_rel[:,0]

            # Apply magnetic effect
            for i in range(self.point_center.shape[0]):
                self.xy_center[i,:] = mat_rot @ self.xy_center[i,:]

            for i in range(self.point_range.shape[0]):
                self.xy_range[i,:] = mat_rot @ self.xy_range[i,:]

            # Setup MAP image --------------------------
            # Convert pixel to meter
            pixel2meter = 0.946981208125

            # Set map image
            img_map = Image.open("./map/Izu_map_mag.png")
            img_list = np.asarray(img_map)
            img_height = img_map.size[0]
            img_width = img_map.size[1]
            img_origin = np.array([722, 749])    # TODO : compute by lat/long of launcher point

            # Define image range 
            img_left =   -1.0 * img_origin[0] * pixel2meter
            img_right = (img_width - img_origin[0]) * pixel2meter
            img_top = img_origin[1] * pixel2meter
            img_bottom = -1.0 * (img_height - img_origin[1]) * pixel2meter

            plt.figure(figsize=(10,8))
            plt.imshow(img_list, extent=(img_left, img_right, img_bottom, img_top))

            # Define color
            color_line = '#ffff33'    # Yellow
            color_circle = 'r'    # Red

            # Set circle object
            ax = plt.axes()

            # plot limit area
            for i in range(self.point_center.shape[0]):
                circle = patches.Circle(xy=self.xy_center[i,:], radius=self.lim_radius,
                                        ec=color_circle, fill=False)
                ax.add_patch(circle)
                plt.plot(self.xy_center[i,0], self.xy_center[i,1], '.', color=color_circle)

            plt.plot(self.xy_range[:,0], self.xy_range[:,1], '--', color=color_line)


    def plot_scatter(self, wind_case):

        # Define computation pattern
        vel_pat = int(wind_case[2])
        dir_pat = int(wind_case[3])

        # Call JudgeInside class
        self.judge_result = np.zeros([dir_pat, vel_pat], dtype=bool)
        judge = JudgeInside()
        judge.set_limit_area(self.xy_range)
        judge.set_limit_circle(self.xy_center, lim_radius=self.lim_radius)

        # Judge landing point is inside limit area or not
        for dir in range(dir_pat):
            for vel in range(vel_pat):
                case = vel * dir_pat + dir
                self.judge_result[dir, vel] = judge.judge_inside(self.drop_point[case,:])

        # Plot scatter graph on MAP
        cmap = plt.get_cmap("Wistia")
        
        for iter in range(vel_pat):
            case_st = iter * dir_pat
            case_en = case_st + dir_pat

            wind_vel = self.wind_cond[case_st, 0]
            label_name = str(wind_vel) + "m/s"

            x_point = np.r_[self.drop_point[case_st:case_en, 0], self.drop_point[case_st, 0]]
            y_point = np.r_[self.drop_point[case_st:case_en, 1], self.drop_point[case_st, 1]]

            plt.plot(x_point, y_point,
                     color=cmap(iter/vel_pat), label = label_name, marker='o')

        # show result
        print(self.judge_result)
        plt.legend()
        plt.show()


    def plot_orbit(var):
        """
        Plot the result

        """

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.plot(var[:,4],var[:,5],var[:,6])

        range_lim = np.max(np.absolute(var[:,4:6]))
        ax.set_xlim(-range_lim,range_lim)
        ax.set_ylim(-range_lim,range_lim)
        ax.set_zlim(0,)

        ax.set_xlabel("X[m]")
        ax.set_ylabel("Y[m]")
        ax.set_zlabel("Up[m]")



if __name__ == "__main__":

    place = 'Izu_land'

    plot_map = PostProcess()
    plot_map.set_map(place)

    plt.show()