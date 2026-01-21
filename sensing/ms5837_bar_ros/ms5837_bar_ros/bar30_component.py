import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from . import ms5837

import time


class BarComponentr:
    def __init__(self):
        
        self.sensor = ms5837.MS5837_30BA() # Default I2C bus is 7
        #self.sensor = ms5837.MS5837_30BA(7) # Specify I2C bus
        #self.sensor = ms5837.MS5837_02BA()
        #self.sensor = ms5837.MS5837_02BA(0)
        #self.sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model and bus

        # We must initialize the self.sensor before reading it
        if not self.sensor.init():
                print("Sensor could not be initialized")
                exit(1)

        # We have to read values from self.sensor to update pressure and temperature
        if not self.sensor.read():
                print("Sensor read failed!")
                exit(1)

        print("Pressure: {} atm {} Torr {} psi".format(
                round( self.sensor.pressure(ms5837.UNITS_atm), 2),
                round( self.sensor.pressure(ms5837.UNITS_Torr), 2),
                round( self.sensor.pressure(ms5837.UNITS_psi), 2),
        ))


        print("Temperature: {} C {} F {} K".format(
                round( self.sensor.temperature(ms5837.UNITS_Centigrade), 2),
                round( self.sensor.temperature(ms5837.UNITS_Farenheit), 2),
                round( self.sensor.temperature(ms5837.UNITS_Kelvin), 2),
        ))

        self.freshwaterDepth = self.sensor.depth() # default is freshwater
        self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
        self.saltwaterDepth = self.sensor.depth() # No nead to read() again
        self.sensor.setFluidDensity(1000) # kg/m^3


        # TODO me
        self.ajust_depth = 0.1 # m
        self.init_fresh_depth = self.freshwaterDepth - self.ajust_depth
        self.init_salt_depth = self.saltwaterDepth - self.ajust_depth

        print("Depth: {} m (freshwater) {} m (saltwater)".format(
                round(self.init_fresh_depth , 3),
                round(self.init_salt_depth , 3),
        ))

        # fluidDensity doesn't matter for altitude() (always MSL air density)
        print("MSL Relative Altitude: {} m".format( self.sensor.altitude() )) # relative to Mean Sea Level pressure in air

        time.sleep(1)


    def pressure_value(self):
        if self.sensor.read():
                hpa_data = self.sensor.pressure()
                psi_data = self.sensor.pressure(ms5837.UNITS_psi)

                # Debag
                #print("Pressure: {} hPa  {} psi".format(
                #        round( hpa_data, 1), # Default is mbar (no arguments)
                #        round( psi_data, 3), # Request psi
                #)+"\n")
        else:
                #print("Sensor read failed!")
                exit(1)

        return hpa_data, psi_data

    def temperature_value(self):
        if self.sensor.read():
                temp_degrees = self.sensor.temperature()
                temp_farenheit = self.sensor.temperature(ms5837.UNITS_Farenheit)
                
                # Debag
                #print("Temperature: {} C  {} F".format(
                #        round(temp_degrees , 2), # Default is degrees C (no arguments)
                #        round(temp_farenheit , 2), # Request Farenheit
                #)+"\n")
        else:
                #print("Sensor read failed!")
                exit(1)

        return temp_degrees, temp_farenheit

    def depth_value(self):
        if self.sensor.read():
                #fresh_depth = self.freshwaterDepth
                #salt_depth = self.saltwaterDepth
                depth_data = self.sensor.depth()

                # Debag
                #print("Depth: {} m (freshwater) {} m (saltwater)".format(
                #        round(freshwater_depth , 3),
                #        round(saltwater_depth , 3),
                #)+"\n")
        else:
                #print("Sensor read failed!")
                exit(1)

        #return fresh_depth, salt_depth
        return depth_data

    def depth_init_error(self):
           return self.init_fresh_depth, self.init_salt_depth


class BarNode(Node):

    def __init__(self):
        super().__init__('bar30_node')
        self.pub_depth = self.create_publisher(Float32, 'bar30/depth', 10)
        self.pub_depth_salt = self.create_publisher(Float32, 'bar30/depth_salt', 10)


        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ms5837_data = BarComponentr()

        self.msg_depth = Float32()
        self.msg_depth_salt = Float32()

        self.init_fresh, self.init_salt = self.ms5837_data.depth_init_error()


    def timer_callback(self):

        depth_data = self.ms5837_data.depth_value()


        # TODO me
        ajust_depth = 0.1 # meter

        #self.msg_depth.data = round(fresh_depth, 3)
        self.msg_depth.data = round(depth_data - ajust_depth - self.init_fresh, 3)
        self.msg_depth_salt.data = round(depth_data - ajust_depth - self.init_salt, 3)

        # self.get_logger().info('Fresh Detph :{} m'.format(self.msg_depth.data))
        
        self.pub_depth.publish(self.msg_depth)
        self.pub_depth_salt.publish(self.msg_depth_salt)


def main(args=None):
    rclpy.init(args=args)
    ms5837_node = BarNode()

    rclpy.spin(ms5837_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ms5837_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
