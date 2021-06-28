import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
ADDR_PRO_LED_RED = 65
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132

# Data Byte Length
LEN_PRO_LED_RED = 1
LEN_PRO_CURRENT_LIMIT = 2
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_PRESENT_POSITION = 4
LEN_PRO_VELOCITY_PROFILE = 4

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

VELOCITY_PROFILE = 100   # motor speed


class LEG:
    def __init__(self, portHandler, motor_ID,  side, position='middle'):
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupBulkRead = GroupBulkRead(portHandler, self.packetHandler)
        self.groupBulkWrite = GroupBulkWrite(portHandler, self.packetHandler)
        self.portHandler = portHandler
        self.motor_ID = motor_ID
        self.side = side
        self.position = position

        # Enabling motors torque
        for i in range(3):
            self.torque(self.motor_ID[i], TORQUE_ENABLE)

            dxl_addparam_result = self.groupBulkRead.addParam(self.motor_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % self.motor_ID[i])
                quit()

        # Defining side correction (for left/right legs)
        if self.side == 'left':
            self.side_corr = 1
        elif self.side == 'right':
            self.side_corr = -1
        else:
            print('Leg side is defined wrong. Please re-define')
            quit()

        # Defining position correction (for front/back legs)
        if self.position == 'front':
            self.pos_corr = np.deg2rad(45)
        elif self.position == 'middle':
            self.pos_corr = 0
        elif self.position == 'back':
            self.pos_corr = np.deg2rad(-45)
        else:
            print('Leg position is defined wrong. Please re-define')
            quit()

    def set_leg_pos(self, pos):
        # pos in tick dimensions

        for i in range(3):
            param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(pos[i])), DXL_HIBYTE(DXL_LOWORD(pos[i])), DXL_LOBYTE(DXL_HIWORD(pos[i])), DXL_HIBYTE(DXL_HIWORD(pos[i]))]
            dxl_addparam_result1 = self.groupBulkWrite.addParam(self.motor_ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position1)

            if dxl_addparam_result1 != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % self.motor_ID[i])
                quit()

        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()

    def motors_angles(self):
        # Check if groupbulkread data of motor is available

        motor_angles = []
        for i in range(3):
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Get present position value
            dxl_getdata_result = self.groupBulkRead.isAvailable(self.motor_ID[i], ADDR_PRO_PRESENT_POSITION,
                                                                 LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % self.motor_ID[i])
                quit()

            dxl_present_position = self.groupBulkRead.getData(self.motor_ID[i], ADDR_PRO_PRESENT_POSITION,
                                                              LEN_PRO_PRESENT_POSITION)
            motor_angles.append(dxl_present_position)
        return motor_angles

    def torque(self, motor_id, enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % motor_id)

    def torque_off(self):
        for i in range(3):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.motor_ID[i], ADDR_PRO_TORQUE_ENABLE, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def loc2deg(self, x, y, z):
        # converts x,y,z in world axis system to engines' degrees.
        # NOTE: while calculating this program assumes:
        # #1 outer leg edge is lower then the body
        # #2 inner leg edge is higher then outer leg edge
        # #3 x is forward, y is upwards and z is outwards
        # #4 y is given in opposite direction (greater y = higher body (=lower leg edge) ).
        # function fixes coordinates according to leg side (left/right)

        a = np.sqrt(x ** 2 + z ** 2)

        psi = np.arctan2(x / a, z / a)

        b = np.sqrt(x ** 2 + y ** 2 + z ** 2)

        l1 = 210.54
        l2 = 372.4

        C = (l1 ** 2 + b ** 2 - l2 ** 2) / (2 * l1 * b)
        D = (l1 ** 2 + l2 ** 2 - b ** 2) / (2 * l1 * l2)

        alpha = np.arcsin(y/b)

        theta = np.arctan2(np.sqrt(1 - C ** 2), C) - alpha
        phi = np.arctan2(np.sqrt(1 - D ** 2), D)

        # rad to deg
        psi_d = np.degrees(psi)
        theta_d = np.degrees(theta)
        phi_d = np.degrees(phi)

        psi_t = psi_d * 2048 / 180  # deg to relative ticks
        theta_t = theta_d * 2048 / 180
        phi_t = phi_d * 2048 / 180

        psi_a = round(2048 + self.side_corr*psi_t)  # relative ticks to absolute ticks (world axis system)
        theta_a = round(2048 + self.side_corr*(theta_t - 512))
        phi_a = round(2048 - self.side_corr*(phi_t - 1536))

        pos = [psi_a, theta_a, phi_a]
        return pos

    def x_forward(self, step_size, resolution):
    # Creates x values for forward movement (one leg at a time).
        front_offset = 100
        back_offset = -1 * front_offset

        if self.position == 'front':
            return range(step_size + front_offset, front_offset, -resolution)
        elif self.position == 'back':
            return range(back_offset, -step_size + back_offset, -resolution)
        else:
            return range(round(step_size/2), -round(step_size/2), -resolution)

    def x_backwards(self, step_size, resolution):
    # Creates x values for forward movement (one leg at a time).
        front_offset = 100
        back_offset = -1 * front_offset
        if self.position == 'front':
            return range(front_offset, step_size + front_offset, resolution)
        elif self.position == 'back':
            return range(-step_size + back_offset, back_offset, resolution)
        else:
            return range(-round(step_size / 2), round(step_size / 2), resolution)

    def pos_fix(self, x_f, z_f):
        x_f_f = []          # stands for x_forward_fixed
        z_f_f = []          # stands for z_forward_fixed

        for i,k in zip(x_f, z_f):
            x_f_f.append(i*np.cos(self.pos_corr) - k*np.sin(self.pos_corr))
            z_f_f.append(i*np.sin(self.pos_corr) + k*np.cos(self.pos_corr))

        return x_f_f, z_f_f





