# scripted/programs/script_basic_grab_steps.py
def forward(mm):
    list_of_steps.append(str(len(str(mm)))+"DRIVE"+str(mm))

def backward(mm):
    list_of_steps.append(str(len(str(mm))+1)+"DRIVE" + str(0-mm))

list_of_steps = ["DRIVE01", "ROTATE01", "LIFT_UP01", "DRIVE01", "DRIVE01", "DRIVE01"]
list_of_steps.append("DONE")