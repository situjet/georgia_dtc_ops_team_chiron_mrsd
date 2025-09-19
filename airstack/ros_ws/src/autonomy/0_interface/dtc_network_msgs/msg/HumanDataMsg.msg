# Custom message for human data containing GPS, compressed image, and raw image
std_msgs/Header header
builtin_interfaces/Time stamp

#Name of the system sending the image data
string system

# GPS data
sensor_msgs/NavSatFix gps_data

#Image Data
#Note: Ether 1 of "compressed_images" or "raw_images" should be populated.
#Its an error is both are empty. 
# Compressed image data
sensor_msgs/CompressedImage[] compressed_images
# Raw image data
sensor_msgs/Image[] raw_images
