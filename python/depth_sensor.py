import ms5837
import time

sensor = ms5837.MS5837_30BA(7) # Default I2C bus is 1 (Raspberry Pi 3)

# We must initialize the sensor before reading it
if not sensor.init():
        with open ("/home/pi/deepsouth/logs/waterdepth.txt", "w") as waterdepth:
                                waterdepth.write("Not Available")
                                waterdepth.close
        with open ("/home/pi/deepsouth/logs/watertemp.txt", "w") as watertemp:
                                watertemp.write("Not Available")
                                watertemp.close
        with open ("/home/pi/deepsouth/logs/waterpressure.txt", "w") as waterpressure:
                                waterpressure.write("Not Available")
                                waterpressure.close
        print ("Water Depth, Temperature, and Pressure not available")
        exit(1)

# Print readings
while True:
        if sensor.read():
                with open ("/home/pi/deepsouth/logs/waterdepth.txt", "w") as waterdepth:
                                waterdepth.write("%s" % (round(sensor.depth(),1)))
                                waterdepth.close
                with open ("/home/pi/deepsouth/logs/watertemp.txt", "w") as watertemp:
                                watertemp.write("%s" % (round(sensor.temperature(),1)))
                                watertemp.close
                with open ("/home/pi/deepsouth/logs/waterpressure.txt", "w") as waterpressure:
                                waterpressure.write("%s" % (round(sensor.pressure(ms5837.UNITS_psi),3)))
                                waterpressure.close
                print ("External sensor recording to logfiles P: %0.3f psi\tT: %0.2f C D: %0.1f m") % (
                sensor.pressure(ms5837.UNITS_psi), # Request psi
                sensor.temperature(), # Default is degrees C (no arguments)
                sensor.depth()) 
                time.sleep(1)

        else:
                print ("External sensor read failed!")
                exit(1)

