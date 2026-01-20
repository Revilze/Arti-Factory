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

        #Camera label
        self.CameraLabel = tk.Label(self.root, text="Camera Settings:", font=self.boldLabelFont)

        
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
        self.StartServoAngleLabel = tk.Label(self.root, text="Start Servo Angle (-125 to 125)", font=self.labelFont)
        self.TextBoxStartServoAngle = tk.Entry(self.root)

        #End Servo Angle
        self.EndServoAngleLabel = tk.Label(self.root, text="End Servo Angle (-125 to 125)", font=self.labelFont)
        self.TextBoxEndServoAngle = tk.Entry(self.root)

        #Servo Increment
        self.ServoIncrementLabel = tk.Label(self.root, text="Servo Increment", font=self.labelFont)
        self.TextBoxServoIncrement = tk.Entry(self.root)

        #Stepper Label
        self.StepperLabel = tk.Label(self.root, text="Stepper Settings:", font=self.boldLabelFont)
        
        #Number of Pictures Per Full Circle
        acceptableStepsPerFullCircleValues = ["2", "4", "8", "16", "32", "50", "64", "100", "128", "256", "512", "800", "1280", "1600", "2560", "3200","6400", "12800"]
        self.StepperNumPicsPerCircleLabel = tk.Label(self.root, text="Number of Pictures Per Full Circle", font=self.labelFont)
         # Create a StringVar to hold the selected number of pictures per full circle
        self.StepperNumPicsPerCircleString = tk.StringVar(self.root)
        self.StepperNumPicsPerCircleString.set("100") # Set the default value
        self.dropdown_menuStepperNumPicsPerCircle = tk.OptionMenu(self.root, self.StepperNumPicsPerCircleString, *acceptableStepsPerFullCircleValues)
    
        #Specmin Label
        self.SpecminLabel = tk.Label(self.root, text="Specmin Information:", font=self.boldLabelFont)

        #Specmin Id
        self.SpecminIdLabel = tk.Label(self.root, text="Specmin Id", font=self.labelFont)
        self.TextBoxSpecminId = tk.Entry(self.root)

        #Specmin Name
        self.SpecminNameLabel = tk.Label(self.root, text="Specmin Name", font=self.labelFont)
        self.TextBoxSpecminName = tk.Entry(self.root)

        #Project Label
        self.ProjectLabel = tk.Label(self.root, text="Project Information:", font=self.boldLabelFont)

        #Project Id
        self.ProjectIdLabel = tk.Label(self.root, text="Project Id", font=self.labelFont)
        self.TextBoxProjectId = tk.Entry(self.root)

        #Project Name
        self.ProjectNameLabel = tk.Label(self.root, text="Project Name", font=self.labelFont)
        self.TextBoxProjectName = tk.Entry(self.root)

        #Project Area
        self.ProjectAreaLabel = tk.Label(self.root, text="Project Area", font=self.labelFont)
        self.TextBoxProjectArea = tk.Entry(self.root)

        
        
        
        #add gui elements for 
        #self.startingServoAngle = 45
        #self.endingServoAngle = 45
        #self.sevoAngleIncrement=5
        
        #make this a drop down menu
        self.numberOfPicturesPerFullCircle = 100 #512 #256 #512 #must divide into 12800 evenly 
        #2, 4, 8, 16, 32, 64, 128, 256, 512, 800, 1600, 512*5
        self.specimenNumber = "2342"
        #self.specimenName = "Pepper corn"
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
        if self.af.cap is not None:
            self.af.cap.release()
        self.IsScanning = True
        print(self.TextBoxStartServoAngle.get())
        print(self.TextBoxEndServoAngle.get())
        print(self.cameraSelectionString.get())
        camera_index = int(self.cameraSelectionString.get())
        #GET FOR SPECIMEN AND PROJECT
        specimenName = self.TextBoxSpecminName.get()
        specimenId = self.TextBoxSpecminId.get()
        projectId = self.TextBoxProjectId.get()
        projectName = self.TextBoxProjectName.get()
        projectArea = self.TextBoxProjectArea.get()   
        #GET FOR SERVO AND STEPPER
        startingServoAngle = float(self.TextBoxStartServoAngle.get())
        print("startingServoAngle: " + str(startingServoAngle))
        endingServoAngle = float(self.TextBoxEndServoAngle.get())
        print("endingServoAngle: " + str(endingServoAngle))
        sevoAngleIncrement=float(self.TextBoxServoIncrement.get())
        numberOfPicturesPerFullCircle = int(self.StepperNumPicsPerCircleString.get()) 
            
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
        nested_path = "/home/oliver/ArtiFactoryPictures/"+projectName+"_"+projectId+"_"+specimenName+"_"+specimenId+"_"+datetime_string+"/"
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
        #Define Geometry
        self.root.geometry("3000x1500")
        
        #Camera
        self.CameraLabel.grid(row=0, column=0)
        self.cameraSelectionLabel.grid(row=1, column=0)
        self.dropdown_menuCameraSelection.grid(row=1, column=1)

        #Servo
        self.ServoLabel.grid(row=3, column=0)
        self.StartServoAngleLabel.grid(row=4, column=0)
        self.TextBoxStartServoAngle.grid(row=4, column=1)
        self.EndServoAngleLabel.grid(row=5, column=0)
        self.TextBoxEndServoAngle.grid(row=5, column=1)
        self.ServoIncrementLabel.grid(row=6, column=0)
        self.TextBoxServoIncrement.grid(row=6, column=1)
        
        # Stepper
        self.StepperLabel.grid(row=7, column=0, pady=20)
        self.StepperNumPicsPerCircleLabel.grid(row=8, column=0, padx=20)
        self.dropdown_menuStepperNumPicsPerCircle.grid(row=8, column=1, padx=20)
        #Specmin
        self.SpecminLabel.grid(row=9, column=0, pady=20)
        self.SpecminNameLabel.grid(row=10, column=0)
        self.TextBoxSpecminName.grid(row=10, column=1)
        self.SpecminIdLabel.grid(row=11, column=0)
        self.TextBoxSpecminId.grid(row=11, column=1)

        #Project
        self.ProjectLabel.grid(row=12, column=0, pady=20)
        self.ProjectNameLabel.grid(row=13, column=0)
        self.TextBoxProjectName.grid(row=13, column=1)
        self.ProjectIdLabel.grid(row=14, column=0)
        self.TextBoxProjectId.grid(row=14, column=1)
        

        

        #Start Scan Button
        self.ButtonStartRun.grid(row=20, column=0, columnspan=2, pady=10)

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