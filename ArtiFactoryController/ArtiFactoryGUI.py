from logging import root
from ArtiFactoryController import ArtiFactory
import time
import cv2
import tkinter as tk
from PIL import Image, ImageTk


class ArtiFactoryGUI():
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Arti-Factory Display")

        image_label = tk.Label(self.root)
        image_label.pack()
        af = ArtiFactory()
        avaliableCameras = af.list_available_cameras(10)
        # Create a Tkinter variable to track the selection
        variable = tk.StringVar(self.root)
        variable.set(avaliableCameras[0]) # Set the default value
        # Create the OptionMenu widget
        dropdown_menu = tk.OptionMenu(self.root, variable, *avaliableCameras)
        dropdown_menu.pack(pady=10) # Add some padding

        startServoAngleLable = tk.Label(self.root, text="Start Servo Angle")
        startServoAngleLable.pack(pady=10)
        # The text box (Entry widget)
        self.TextBoxStartServoAngle = tk.Entry(self.root)
        self.TextBoxStartServoAngle.pack(pady=5)
        
        endServoAngleLable = tk.Label(self.root, text="End Servo Angle")
        endServoAngleLable.pack(pady=10)
        # The text box (Entry widget)
        self.TextBoxEndServoAngle = tk.Entry(self.root)
        self.TextBoxEndServoAngle.pack(pady=5)

        #add gui elements for 
        #self.startingServoAngle = 45
        #self.endingServoAngle = 45
        self.sevoAngleIncrement=5
        
        #make this a drop down menu
        self.numberOfPicturesPerFullCircle = 100 #512 #256 #512 #must divide into 12800 evenly 
        #2, 4, 8, 16, 32, 64, 128, 256, 512, 800, 1600, 512*5
        self.specimenNumber = "2342"
        self.specimenName = "Pepper corn"
        self.specimenDescription = "This is a pepper corn specimen."
        self.projectNumber = "1234"
        
        #Add this last where does the user want to save pictures

        ButtonStartRun = tk.Button(self.root, text="Start Run", command=self.whenStartButtonClicked)
        ButtonStartRun.pack(pady=10)

    def whenStartButtonClicked(self):
        print(self.TextBoxStartServoAngle.get())
        print(self.TextBoxEndServoAngle.get())

        return
    
    def runGUILoop(self):
        self.root.mainloop()
       

    


def main():
    AFG = ArtiFactoryGUI()
    # Run the Tkinter event loop
    AFG.runGUILoop()
    '''
    #Add a start capture button and run the following code updating any parameters from the GUI
    af.setup_camera(camera_index)

    if(os.path.exists("pictures/")==False):
        os.makedirs("pictures/")
    
    for i in range(2):
        print("camera is warming up: " + str(2-i))
        filename = "pictures/warmup"+str(i)+".png"
        af.capture_webcam_image(filename)
        time.sleep(0.25)
        
    # Check if the camera opened successfully
    if not af.cap.isOpened():
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
        af.moveServo(curServoAngle)
        for currentStepLocation in range(numberOfPicturesPerFullCircle):
            filename = nested_path+specimenName+"_"+str(curServoAngle)+"_"+str(currentStepLocation)+".png"
            af.capture_webcam_image(filename, numberOfFramesToDiscard)
            af.moveMotor(12800/numberOfPicturesPerFullCircle)
            original_image = Image.open(filename)
            current_tk_image = ImageTk.PhotoImage(original_image)
            image_label.config(image=current_tk_image)
            image_label.pack()
            root.update()

    af.cap.release()
    '''
    return

if __name__ == "__main__":
    main()