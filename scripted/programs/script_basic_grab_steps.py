# scripted/programs/script_basic_grab_steps.py
def move(mm):
    list_of_steps.append(str(len(str(mm)))+"DRIVE"+str(mm))

def rotate(degrees):
    list_of_steps.append(str(len(str(degrees)))+"ROTATE"+str(degrees))

def lift_up():
    list_of_steps.append("0"+"LIFT_UP")

def lift_down():
    list_of_steps.append("0"+"LIFT_DOWN")

def grab():
    list_of_steps.append("0"+"VACUUM_ON")

def release():
    list_of_steps.append("0"+"VACUUM_OFF")

def align_block():
    list_of_steps.append("0ALIGN_BLOCK")


def order():
    move(375)
    grab()
    release()
    rotate(27)
    move(880)
    lift_up()
    move(500)
    lift_down()
    grab()
    lift_up()
    move(-800)
    for i in range(2):
        rotate(45)
    move(1100)
    lift_down()
    release()
    move(-200)
    for i in range(1):
        rotate(-35)
    move(800)
    rotate(-3)
    # move(300)
    # move(100)
    # rotate(-10)
    # rotate(-4)
    # align_block()


list_of_steps = []
order()
list_of_steps.append("0DONE")