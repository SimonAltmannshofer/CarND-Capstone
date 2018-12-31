import matplotlib.pyplot as plt
import csv

with open('records/driving_log.csv', 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    count = 0
    for row in reader:
        if count == 0:
            for variable in row:
                exec(variable + " = []")
            count += 1
        else:
            count += 1
