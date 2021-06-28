import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from leg_class import LEG

class HEXAPOD:
    def __init__(self, legs):
        self.legs = legs

    def pos_fix(self, pos, resolution, parts=2):
        pos_fix = []
        max_samples = 0

        for leg_num in range(6):
            present_pos = self.legs[leg_num].motors_angles()
            des_pos = self.legs[leg_num].loc2deg(pos[0][leg_num][0], pos[1][leg_num][0], pos[2][leg_num][0])
            max_values = max(abs(present_pos[0] - des_pos[0]), abs(present_pos[1] - des_pos[1]),
                              abs(present_pos[2] - des_pos[2]))
            pos_fix.append(np.ndarray.tolist(np.linspace(present_pos, des_pos, int(max_values / resolution)).astype(int)))
            if max_values > max_samples:
                max_samples = max_values

        if parts == 1:
            for it in range(int(max_samples/resolution)):
                for leg_num in range(6):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        elif parts == 2:
            for it in range(int(max_samples / resolution)):
                for leg_num in (0, 2, 4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (1, 3, 5):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        elif parts == 3:
            for it in range(int(max_samples / resolution)):
                for leg_num in (0, 5):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (1, 4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (2, 3):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        else:
            for leg_num in (0,3,5,2,1,4):
                for it in range(int(max_samples / resolution)):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])

    def walk(self, direction, y, z, step_size, resolution, step_height, step_dist, steps_num):
        # Moves the robot forward according to received parameters.

        samples = int(step_size/resolution)

        x_f = []                                            # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        y_f = []                                            # array of y value sets for six legs (etc.)
        z_f = []                                            # array of z value sets for six legs (etc.)

        x_b = []                                            # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
        y_b = []                                            # etc..
        z_b = []

        for num, leg in zip(range(6), self.legs):  # creating value sets
            if direction == 'forward':
                x_f.append(leg.x_forward(step_size, resolution))  # calculating x values for forward movement
            elif direction == 'backwards':
                x_f.append(leg.x_backwards(step_size, resolution))    # calculating x values for backwards movement
            else:
                print('walking direction is defined wrong (should be "forward"/"backwards" only). Please re-define')
                quit()

            y_f.append([y] * samples)
            z_f.append([z] * samples)

            x_f_f, z_f_f = leg.pos_fix(x_f[num], z_f[num])  # rotating x and z values to leg axis system
            x_f[num] = x_f_f
            z_f[num] = z_f_f

            x_b.append(np.flip(x_f[num]))  # creating values set for x backwards movement (opposite of x_f)

            y_b1 = np.linspace(y, y - step_height, int(samples / 2))
            y_b2 = np.linspace(y - step_height, y, int(samples / 2))
            y_b.append(np.ndarray.tolist(np.append(y_b1, y_b2)))

            z_b1 = np.linspace(z_f[num][-1], z_f[num][-1] - step_dist, int(samples / 2))
            z_b2 = np.linspace(z_f[num][-1] - step_dist, z_f[num][0], int(samples / 2))
            z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))

            if num % 2 != 0:                                # switching double legs (0,2,4) values
                x_f[num],x_b[num] = x_b[num],x_f[num]
                y_f[num],y_b[num] = y_b[num],y_f[num]
                z_f[num],z_b[num] = z_b[num],z_f[num]

            if direction == 'backwards':
                x_f[num], x_b[num] = x_b[num], x_f[num]
                y_f[num], y_b[num] = y_b[num], y_f[num]
                z_f[num], z_b[num] = z_b[num], z_f[num]

        self.pos_fix([x_f, y_f, z_f], 8, 3)

        for step in range(steps_num):
            print('\nFirst half cycle')
            for it in range(samples-1):
                for leg_num,i,j,k in zip(range(6),x_f,y_f,z_f):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]),round(j[it]),round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.01)

            print('\nSecond half cycle')
            for it in range(samples-1):
                for leg_num,i,j,k in zip(range(6),x_b,y_b,z_b):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]),round(j[it]),round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.001)

    def spin(self, resolution = 4, step_size = 60, y = 175, z = 225, step_height = 20, step_dist = 20, steps_num = 12):
        x_f = [[],[],[],[],[],[]]

        x_f[0] = range(step_size, 0, -resolution)
        x_f[1] = range(round(step_size / 2), -round(step_size / 2), -resolution)
        x_f[2] = range(0, -step_size, -resolution)
        x_f[3] = range(0, step_size, resolution)
        x_f[4] = range(-round(step_size / 2), round(step_size / 2), resolution)
        x_f[5] = range(-step_size, 0, resolution)

        y_f = []  # array of y value sets for six legs (etc.)
        z_f = []  # array of z value sets for six legs (etc.)

        x_b = []  # array of x value sets for six legs. 'b' stands for 'backwards motion'
        y_b = []  # etc..
        z_b = []

        samples = int(step_size/resolution)

        for num, leg in zip(range(6), self.legs):  # creating value sets

            y_f.append([y] * samples)
            z_f.append([z] * samples)

            x_b.append(np.flip(x_f[num]))  # creating values set fot x backwards movement (opposite of x_f)

            y_b1 = np.linspace(y, y - step_height, int(samples / 2))
            y_b2 = np.linspace(y - step_height, y, int(samples / 2))
            y_b.append(np.ndarray.tolist(np.append(y_b1, y_b2)))

            z_b1 = np.linspace(z_f[num][-1], z_f[num][-1] - step_dist, int(samples / 2))
            z_b2 = np.linspace(z_f[num][-1] - step_dist, z_f[num][0], int(samples / 2))
            z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))

            if num % 2 != 0:
                x_temp = x_f[num]
                y_temp = y_f[num]
                z_temp = z_f[num]

                x_f[num] = x_b[num]
                y_f[num] = y_b[num]
                z_f[num] = z_b[num]

                x_b[num] = x_temp
                y_b[num] = y_temp
                z_b[num] = z_temp

        self.pos_fix([x_f, y_f, z_f], 5, 6)

        for step in range(steps_num):
            print('\nFirst half cycle')
            for it in range(samples - 1):
                for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.001)

            print('\nSecond half cycle')
            for it in range(samples - 1):
                for leg_num, i, j, k in zip(range(6), x_b, y_b, z_b):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.001)

    def roll(self, angle, resolution, go_back = 0):
        dist_mid = 174.25
        dist_non_mid = 139.26
        angles = range(0, abs(angle) + 1, resolution)
        front_offset = 120
        back_offset = front_offset * -1
        y_const = 200
        z_const = 250

        x_a = [[],[],[],[],[],[]]
        y_a = [[],[],[],[],[],[]]
        z_a = [[],[],[],[],[],[]]

        for num in range(6):
            if num == 1 or num == 4:
                dist = dist_mid
            else:
                dist = dist_non_mid

            if num < 3:
                for ang in angles:
                    y_a[num].append(y_const+(dist*np.sin(np.deg2rad(ang))))
                    z_a[num].append(z_const-(dist-dist*np.cos(np.deg2rad(ang))))
            else:
                for ang in angles:
                    y_a[num].append(y_const-(dist*np.sin(np.deg2rad(ang))))
                    z_a[num].append(z_const+(dist-dist*np.cos(np.deg2rad(ang))))

        for num in range(6):
            for i in range(len(y_a[num])):
                if self.legs[num].position == 'front':
                    x_a[num].append(front_offset)
                elif self.legs[num].position == 'back':
                    x_a[num].append(back_offset)
                else:
                    x_a[num].append(0)
            x_a_f,z_a_f = self.legs[num].pos_fix(x_a[num],z_a[num])
            x_a[num] = x_a_f
            z_a[num] = z_a_f

        if angle < 0:
            for i in range(3):
                x_a[i],x_a[3+i] = x_a[3+i],x_a[i]
                y_a[i],y_a[3+i] = y_a[3+i],y_a[i]
                z_a[i],z_a[3+i] = z_a[3+i],z_a[i]

        self.pos_fix([x_a, y_a, z_a], 5, 3)

        for it in range(len(angles)):
            for num in range(6):
                self.legs[num].set_leg_pos(self.legs[num].loc2deg(round(x_a[num][it]),round(y_a[num][it]),round(z_a[num][it])))
                print(round(x_a[num][it]),round(y_a[num][it]),round(z_a[num][it]))

        if go_back == 0:
            self.pos_fix([x_a, y_a, z_a], 8, 1)

    def pitch(self, angle, resolution, go_back = 0):
        dist_mid = 0
        dist_non_mid = 127.3
        angles = range(0, abs(angle) + 1, resolution)
        front_offset = 200
        back_offset = front_offset * -1
        y_const = 200
        z_const = 250

        x_a = [[],[],[],[],[],[]]
        y_a = [[],[],[],[],[],[]]
        z_a = [[],[],[],[],[],[]]

        for num in range(6):
            if num == 1 or num == 4:
                dist = dist_mid
            else:
                dist = dist_non_mid

            if num == 2 or num == 5:
                for ang in angles:
                    y_a[num].append(y_const+(dist*np.sin(np.deg2rad(ang))))
                    z_a[num].append(z_const-(dist-dist*np.cos(np.deg2rad(ang))))
            elif num == 0 or num == 3:
                for ang in angles:
                    y_a[num].append(y_const-(dist*np.sin(np.deg2rad(ang))))
                    z_a[num].append(z_const+(dist-dist*np.cos(np.deg2rad(ang))))
            else:
                for ang in angles:
                    y_a[num].append(y_const + 15)
                    z_a[num].append(z_const + 25)

        for num in range(6):
            for i in range(len(y_a[num])):
                if self.legs[num].position == 'front':
                    x_a[num].append(front_offset)
                elif self.legs[num].position == 'back':
                    x_a[num].append(back_offset)
                else:
                    x_a[num].append(0)

            x_a_f,z_a_f = self.legs[num].pos_fix(x_a[num],z_a[num])
            x_a[num] = x_a_f
            z_a[num] = z_a_f

        if angle < 0:
            for i in [0,3]:
                x_a[i], x_a[2 + i] = x_a[2 + 1], x_a[i]
                y_a[i], y_a[2 + i] = y_a[2 + i], y_a[i]
                z_a[i], z_a[2 + i] = z_a[2 + i], z_a[i]

        self.pos_fix([x_a, y_a, z_a], 5, 3)

        for it in range(len(angles)):
            for num in range(6):
                self.legs[num].set_leg_pos(self.legs[num].loc2deg(round(x_a[num][it]),round(y_a[num][it]),round(z_a[num][it])))
                print(round(x_a[num][it]),round(y_a[num][it]),round(z_a[num][it]))

        if go_back == 0:
            self.pos_fix([x_a, y_a, z_a], 8, 1)

    def yaw(self, angle, resolution, go_back = 0): # angle < 40!
        step_size = angle*5
        corr = 1
        if angle<0:
            step_size = abs(step_size)
            corr = -1
        y = 200
        z = 225
        # d_z = 174.25

        x_f = [[], [], [], [], [], []]

        for i in range(3):
            x_f[i] = range(0, -step_size*corr, -resolution*corr)
        for i in  range(3,6):
            x_f[i] = range(0, step_size*corr, resolution*corr)

        y_f = []  # array of y value sets for six legs (etc.)
        z_f = []  # array of z value sets for six legs (etc.)

        samples = int(step_size / resolution)

        for num in range(6):  # creating value sets
            y_f.append([y] * samples)
            z_f_f  = []
            for x in x_f[num]:
                z_f_f.append(np.sqrt(z**2 - x**2))
            z_f.append(z_f_f)

        self.pos_fix([x_f, y_f, z_f], 5, 3)

        for it in range(samples - 1):
            for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
            time.sleep(0.001)

        if go_back == 0:
            self.pos_fix([x_f, y_f, z_f], 8, 1)