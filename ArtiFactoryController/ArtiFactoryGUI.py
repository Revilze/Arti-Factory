#from logging import root
from ArtiFactoryController import ArtiFactory
import time, os 
from datetime import datetime
import cv2
import tkinter as tk
from PIL import Image, ImageTk

def float_range(start, stop, step):
        current = start
        while current < stop:
            yield current
            current += step

class ArtiFactoryGUI():
    def __init__(self):
        self.IsScanning = False
        self.IsCameraSetup = False
        self.boldLabelFont = ("Helvetica", 18, "bold")
        self.labelFont = ("Helvetica", 18)

        self.root = tk.Tk()
        self.root.title("Arti-Factory Display")

        self.image_label = tk.Label(self.root)
        self.af = ArtiFactory()
        
        #camera selection dropdown
        self.avaliableCameras = self.af.list_available_cameras(10)
        self.cameraSelectionLabel = tk.Label(self.root, text="Camera Selection:", font=self.labelFont)
         # Create a StringVar to hold the selected camera index
        self.cameraSelectionString = tk.StringVar(self.root)
        #self.cameraSelectionString.set(self.avaliableCameras[0]) # Set the default value
        self.dropdown_menuCameraSelection = tk.OptionMenu(self.root, self.cameraSelectionString, *self.avaliableCameras, command=self.whenCameraSelectionChanged)
        self.dropdown_menuCameraSelection.bind("<Button-1>", self.update_CameraSelection_values)
        #Servo Label
        self.ServoLabel = tk.Label(self.root, text="Servo Settings:", font=self.boldLabelFont)

        #Start Servo Angle
        self.StartServoAngleLabel = tk.Label(self.root, text="Start Servo Angle", font=self.labelFont)
        self.TextBoxStartServoAngle = tk.Entry(self.root)

        #End Servo Angle
        self.EndServoAngleLabel = tk.Label(self.root, text="End Servo Angle", font=self.labelFont)
        self.TextBoxEndServoAngle = tk.Entry(self.root)

        #Servo Increment
        self.ServoIncrementLabel = tk.Label(self.root, text="Servo Increment", font=self.labelFont)
        self.TextBoxServoIncrement = tk.Entry(self.root)

        #Stepper Label
        self.StepperLabel = tk.Label(self.root, text="Stepper Settings:", font=self.boldLabelFont)
        #Stepper Number of Pictures Per Full Circle
        acceptableStepsPerFullCircleValues = ["2", "4", "8", "16", "32", "50", "64", "100", "128", "256", "512", "800", "1280", "1600", "2560", "3200","6400", "12800"]
        self.StepperNumPicsPerCircleLabel = tk.Label(self.root, text="Stepper Steps Per Full Circle")
        self.StepperNumPicsPerCircleString = tk.StringVar(self.root)
        self.StepperNumPicsPerCircleString.set("100") # Set the default value
        self.dropdown_menuStepperNumPicsPerCircle = tk.OptionMenu(self.root, self.StepperNumPicsPerCircleString, *acceptableStepsPerFullCircleValues)

        #add gui elements for 
        #self.startingServoAngle = 45
        #self.endingServoAngle = 45
        #self.sevoAngleIncrement=5
        
        #make this a drop down menu
        self.numberOfPicturesPerFullCircle = 100 #512 #256 #512 #must divide into 12800 evenly 
        #2, 4, 8, 16, 32, 64, 128, 256, 512, 800, 1600, 512*5
        self.specimenNumber = "2342"
        self.specimenName = "Pepper corn"
        self.specimenDescription = "This is a pepper corn specimen."
        self.projectNumber = "1234"
        self.numberOfFramesToDiscard = 4
        
        #Add this last where does the user want to save pictures

        self.ButtonStartRun = tk.Button(self.root, text="Start Scan", command=self.whenStartButtonClicked)
        
    def update_frame(self):
        if self.IsScanning==False and self.IsCameraSetup==True:
            imagetk = self.af.getImage() 
            if imagetk is not None:
                self.image_label.imgtk = imagetk
                self.image_label.configure(image=imagetk)
                self.image_label.after(20, self.update_frame)

    def whenCameraSelectionChanged(self, selection):
        self.IsCameraSetup=False
        if self.af.cap is not None:
            self.af.cap.release()
        camera_index = int(self.cameraSelectionString.get())
        self.af.setup_camera(int(camera_index))
        self.IsCameraSetup=True
        self.update_frame()
        
    def update_CameraSelection_values(self, event):
        self.IsCameraSetup=False
        if self.af.cap is not None:
            self.af.cap.release()
        cameraList = self.af.list_available_cameras(10)
        menu = self.dropdown_menuCameraSelection['menu']

        # 2. Delete all existing options
        menu.delete(0, tk.END)
        
        # 3. Add the new options
        for option in cameraList:
            # The command binds the option value back to the StringVar
            menu.add_command(label=option, command=tk._setit(self.cameraSelectionString, option, self.whenCameraSelectionChanged))


    def whenStartButtonClicked(self):
        self.IsScanning = True
        print(self.TextBoxStartServoAngle.get())
        print(self.TextBoxEndServoAngle.get())
        print(self.cameraSelectionString.get())
        camera_index = int(self.cameraSelectionString.get())
        specimenName = self.specimenName
        startingServoAngle = int(self.TextBoxStartServoAngle.get())
        endingServoAngle = int(self.TextBoxEndServoAngle.get())
        sevoAngleIncrement=self.sevoAngleIncrement
        numberOfPicturesPerFullCircle = self.numberOfPicturesPerFullCircle  
            
        #Add a start capture button and run the following code updating any parameters from the GUI
        self.af.setup_camera(camera_index)
    

        if(os.path.exists("pictures/")==False):
            os.makedirs("pictures/")
        
        for i in range(2):
            print("camera is warming up: " + str(2-i))
            filename = "pictures/warmup"+str(i)+".png"
            self.af.capture_webcam_image(filename)
            time.sleep(0.25)
            
        # Check if the camera opened successfully
        if not self.af.cap.isOpened():
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
            self.af.moveServo(curServoAngle)
            for currentStepLocation in range(numberOfPicturesPerFullCircle):
                filename = nested_path+specimenName+"_"+str(curServoAngle)+"_"+str(currentStepLocation)+".png"
                self.af.capture_webcam_image(filename, numberOfFramesToDiscard)
                self.af.moveMotor(12800/numberOfPicturesPerFullCircle)
                original_image = Image.open(filename)
                current_tk_image = ImageTk.PhotoImage(original_image)
                self.image_label.config(image=current_tk_image)
                self.root.update()

        self.af.cap.release()
        self.IsScanning = False
        return
    
    def runGUILoop(self):
        self.root.geometry("3000x1500")
        self.cameraSelectionLabel.grid(row=0, column=0)
        self.dropdown_menuCameraSelection.grid(row=0, column=1)

        #Servo
        self.ServoLabel.grid(row=3, column=0)
        self.StartServoAngleLabel.grid(row=4, column=0)
        self.TextBoxStartServoAngle.grid(row=4, column=1)
        self.EndServoAngleLabel.grid(row=5, column=0)
        self.TextBoxEndServoAngle.grid(row=5, column=1)
        self.ServoIncrementLabel.grid(row=6, column=0)
        self.TextBoxServoIncrement.grid(row=6, column=1)
        #Stepper
        self.StepperLabel.grid(row=7, column=0, pady=20)
        self.StepperNumPicsPerCircleLabel.grid(row=8, column=0, padx=20)
        self.dropdown_menuStepperNumPicsPerCircle.grid(row=8, column=1, padx=20)

        #Start Scan Button
        self.ButtonStartRun.grid(row=9, column=0, columnspan=2, pady=10)

        #empty lable to take up space at the bottom of the screen so other components do not move
        emptyLabel = tk.Label(self.root, text="")
        emptyLabel.grid(row=10, column=0, pady=20)

        #Image display
        self.image_label.config(borderwidth=2, relief="solid")
        self.image_label.grid(row=0, column=3, rowspan=10, padx=20)
        
        
        self.root.grid_columnconfigure(3, weight=1) 

        self.root.mainloop()
       
        


def main():
    AFG = ArtiFactoryGUI()
    # Run the Tkinter event loop
    AFG.runGUILoop()
    return

if __name__ == "__main__":
    main()