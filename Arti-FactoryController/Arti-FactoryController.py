#using pySerial pip install pyserial
import os
from datetime import datetime
import serial
import time
import cv2
import tkinter as tk
from PIL import Image, ImageTk

#configuration of the run
startingServoAngle = 45
endingServoAngle = 45
sevoAngleIncrement=5
numberOfPicturesPerFullCircle = 100 #512 #256 #512 #must divide into 12800 evenly 
#2, 4, 8, 16, 32, 64, 128, 256, 512, 800, 1600, 512*5
specimenName = "Pepper corn"

# Configure the serial port
# Replace 'COM3' with the appropriate port for your system (e.g., '/dev/ttyUSBS0' on Linux/macOS)
# Ensure the baudrate matches your device's configuration
port_name = '/dev/ttyACM0' 
baud_rate = 115200

def float_range(start, stop, step):
        current = start
        while current < stop:
            yield current
            current += step


class TiltTurn():
    def __init__(self):
        self.cap = None
        try:
            # Open the serial port
            self.ser = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                timeout=1  # Read timeout in seconds
            )
            print(f"Serial port {port_name} opened successfully.")
            time.sleep(4)
        except serial.SerialException as e:
            print(f"Error opening or communicating with serial port: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            # Close the serial port if it was opened
            if 'ser' in locals() and self.ser.is_open:
                self.ser.close()
                print(f"Serial port {port_name} closed.")
                exit()    # Open the serial port
        
    

    def readData(self, amountToRead:int, timeout=2):
        self.ser.timeout = timeout
        received_data = self.ser.read(amountToRead) 
        return received_data

    def moveServo(self, angle):
        data_to_send = b"S"+str(angle).encode("utf-8")+b"\n"  # 'b' prefix for bytes
        print(data_to_send)
        self.ser.write(data_to_send)
        secondsToSleep = 2
        print("second to sleep " + str(secondsToSleep))
        time.sleep(secondsToSleep)
        received_data = self.readData(250, 1+int(0.025*angle/1000)) 
        if received_data:
            print(f"Received: {received_data.decode().strip()}") # Decode for printing
        else:
            print("No data received within the timeout period.")

    def moveMotor(self, steps):
        data_to_send = b"m"
        if steps >=0:
            data_to_send+= b"+"
        data_to_send+=str(steps).encode("utf-8")
        data_to_send+=b"\n"  # 'b' prefix for bytes
        print(data_to_send)
        self.ser.write(data_to_send)
        secondsToSleep = 0.25+0.00025* abs(steps)
        print("second to sleep " + str(secondsToSleep))
        time.sleep(secondsToSleep)
        received_data = self.readData(250) 
        if received_data:
            print(f"Received: {received_data.decode().strip()}") # Decode for printing
        else:
            print("No data received within the timeout period.")     
    
    def list_available_cameras(self, max_index_to_check=10):
        """
        Attempts to open cameras with indices from 0 up to max_index_to_check-1
        and returns a list of indices for which a camera was successfully opened.
        """
        available_cameras = []
        for i in range(max_index_to_check):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                available_cameras.append(i)
                cap.release()  # Release the camera after checking
        return available_cameras
    
    def capture_webcam_image(self, filename="captured_image.jpg", numberOfFramesToDiscard=4):
        # Flush the buffer by reading and discarding a few frames
        # The number of frames to discard might need adjustment based on your webcam
        for _ in range(numberOfFramesToDiscard):  # Discard frames to ensure a fresh one
            self.cap.grab()
        # Read a frame from the camera
        ret, frame = self.cap.read()
        if ret:
            # Save the captured frame as an image
            cv2.imwrite(filename, frame)
            print(f"Image saved successfully as {filename}")
        else:
            print("Error: Could not read frame from camera.")
        
def main():
    root = tk.Tk()
    root.title("Image Display")

    image_label = tk.Label(root)
    image_label.pack()
    tt = TiltTurn()
    avaliableCameras = tt.list_available_cameras(5)
    print(avaliableCameras)
    camera_index = 2 #avaliableCameras[-1]
    tt.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    cv2.createAlignMTB()
    
    print(tt.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    tt.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75) # Set to manual exposure (0.75 is a common value for this)
    print(tt.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    print(tt.cap.get(cv2.CAP_PROP_EXPOSURE))
    tt.cap.set(cv2.CAP_PROP_EXPOSURE, 60*8) #120 works good for the microscope #85 is no rolling bands in background for voCam 
    print(tt.cap.get(cv2.CAP_PROP_EXPOSURE))
    
    print(tt.cap.get(cv2.CAP_PROP_AUTO_WB))
    #tt.cap.set(cv2.CAP_PROP_AUTO_WB, 1.0)

    # Set the desired frame width and height
    # Common resolutions include:
    # 640x480, 1280x720 (HD), 1920x1080 (Full HD)
    desired_width = 1920
    desired_height = 1080

    tt.cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    tt.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

    # Verify if the resolution was set (optional)
    # Note: The actual resolution might be different if the camera doesn't support the requested size.
    actual_width = int(tt.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(tt.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Actual frame resolution: {actual_width}x{actual_height}")


    if(os.path.exists("pictures/")==False):
        os.makedirs("pictures/")
    for i in range(2):
        print("camera is warming up: " + str(2-i))
        filename = "pictures/warmup"+str(i)+".png"
        tt.capture_webcam_image(filename)
        time.sleep(0.25)
        
    # Check if the camera opened successfully
    if not tt.cap.isOpened():
        print(f"Error: Could not open camera with index {camera_index}.")
        return
    

    
    #this clears the camera buffer and ensures the picture taken was the correct picture for the location
    numberOfFramesToDiscard=4
    # Get the current datetime object
    now = datetime.now()
    # Format the datetime object into a string
    # Example: "YYYY-MM-DD HH:MM:SS"
    datetime_string = now.strftime("%Y_%m_%d_%H_%M_%S")
    print(datetime_string)
    nested_path = "~/Pictures/Arti-Factory/"+specimenName+"_"+datetime_string+"/"
    os.makedirs(nested_path)

    servoStopsString="servo stops: "
    for i in float_range(startingServoAngle, endingServoAngle+sevoAngleIncrement, sevoAngleIncrement):
        servoStopsString+=str(i)+", "
    print(servoStopsString)

    for curServoAngle in float_range(startingServoAngle, endingServoAngle+sevoAngleIncrement, sevoAngleIncrement):
        tt.moveServo(curServoAngle)
        for currentStepLocation in range(numberOfPicturesPerFullCircle):
            filename = nested_path+specimenName+"_"+str(curServoAngle)+"_"+str(currentStepLocation)+".png"
            tt.capture_webcam_image(filename, numberOfFramesToDiscard)
            tt.moveMotor(12800/numberOfPicturesPerFullCircle)
            original_image = Image.open(filename)
            current_tk_image = ImageTk.PhotoImage(original_image)
            image_label.config(image=current_tk_image)
            image_label.pack()
            root.update()

    tt.cap.release()
    #root.mainloop()
    return

if __name__ == "__main__":
    main()