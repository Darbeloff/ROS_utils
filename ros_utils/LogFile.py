
import rospy
import numpy as np

class LogFile:
    """
    A handy class to open and deal with logging files. Use when you want to update a csv over time. Mostly threadsafe.
    """
    def __init__(self, filename, headers):
        #headers should be an array of headers
        #filename should just be a name, string
        self.filename = filename
        self.headers = headers
        with open(self.filename, "w") as file:
            file.write("time")
            for header in headers:
                file.write(",")
                file.write(header)

            file.write('\n')

    def log(self, time, columns):
        #call this every time you want to log, give it the ros timestamp
        try:
            assert(len(columns) == len(self.headers))
        except AssertionError:
            rospy.logwarn("Attempting to log an inconsistant number of values")
            return

        with open(self.filename, "a") as file:
            file.write(str(time))
            for col in columns:
                file.write(",")
                file.write(str(col))

            file.write('\n')