# Message Type Encapsulating GPS data retrieved from Teensy
# The raw values of all these measurements were provided by the sensor in
# integer format. The publisher takes care of applying the proper scaling
# to each of the values before publishing them on a topic.

# String: timestamp
#           -represents the GPS module time of measurement
#           Format: hours:minutes:seconds.milliseconds
string timestamp

# Float64 latitude, longitude
#           -self explanatory
#           Format: degrees
float64 latitude
float64 longitude

# Float64 altitude
#           -height about the mean sea level
#           Format: meters
float64 altitude

# Float64 ground_speed
#           -how fast the GPS is translating over the ground (2D)
#           Format: meters/second
float64 ground_speed

# Float64 motion_heading
#           -represents the heading of GPS motion with respect to magnetic north
#           Format: degrees
float64 motion_heading

# Float64 horizontal_accuracy
#           -represents the radius around the position reading in which the true position of the GPS resides
#           Format: meters
float64 horizontal_accuracy