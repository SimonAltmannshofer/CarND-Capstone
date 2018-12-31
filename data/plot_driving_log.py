import matplotlib.pyplot as plt
import csv

with open('./records/driving_log.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    count = 0
    for row in reader:
        if count == 0:
            variables = row
            for variable in variables:
                exec(variable + " = []")
            count += 1
        else:
            for i in range(len(row)):
                exec(variables[i] + ".append(" + row[i] + ")")
            count += 1

plt.subplot(411)
plt.plot(throttle, label='throttle')
plt.hold
plt.plot(throttle_des, label='throttle_des')
plt.grid(True)
plt.legend()

plt.subplot(412)
plt.plot(brake, label='brake')
plt.hold
plt.plot(brake_des, label='brake_des')
plt.legend()

plt.subplot(413)
plt.plot(v, label='v')
plt.hold
plt.plot(v_des, label='v_des')
plt.legend()

plt.subplot(414)
plt.plot(v_d, label='v_d')
plt.hold
plt.plot(v_d_des, label='v_d_des')
plt.legend()

plt.show()