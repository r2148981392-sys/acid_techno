import matplotlib.pyplot as plt
import numpy as np
import time

time_points = []
temp_points = []

start_time = time.time() # start of program (s)
last_sample = time.time() # time of last sample (s)
heater_active = False # heater is running or off
heater_lower_threshold = 0 # temp at which heater activates (°C)
heater_upper_threshold = 10 # temp at which heater deactivates (°C)
temp = 20 # current temperature (°C)
temp_rate = -4 # rate of natural temp change (s)
heating_rate = 6 # rate of temp change with heating active (s)

while time.time() - start_time < 30:
    if time.time() - last_sample >= 0.1:
        print("Calculating new step")

        if heater_active:
            temp += heating_rate * (time.time() - last_sample)
            if temp >= heater_upper_threshold:
                heater_active = False
        else:
            temp += temp_rate * (time.time() - last_sample)
            if temp <= heater_lower_threshold:
                heater_active = True

        last_sample = time.time()
        time_points.append(last_sample - start_time)
        temp_points.append(temp)

        print(f"({temp_points[-1]}, {time_points[-1]})")

    time.sleep(0.01)

plt.plot(time_points, temp_points)
plt.show()