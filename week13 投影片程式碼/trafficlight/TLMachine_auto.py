import time
from statemachine import StateMachine, State

### Traffic Light StateMachine
class TrafficLightMachine(StateMachine):
     "A traffic light machine"
     
     ### defined three States
     green = State('Green', initial=True)
     yellow = State('Yellow')
     red = State('Red')

     ### defined transition
     cycle = (
         green.to(yellow)
         | yellow.to(red)
         | red.to(green)
     )

     def on_enter_green(self):
         print("Enter the green.")
         time.sleep(50)
         self.cycle()

     def on_exit_green(self):
         print("Exit the green")
         
     def on_enter_yellow(self):
         print("Enter the yellow")
         time.sleep(10)
         self.cycle()

     def on_exit_yellow(self):
         print("Exit the yellow")
         
     def on_enter_red(self):
         print("Enter the red")
         time.sleep(30)
         self.cycle()

     def on_exit_red(self):
         print("Exit the red")
     

sm = TrafficLightMachine()

current_state = None
while True:
  if current_state is None:
    current_state = sm.current_state
    print("current: ", current_state)
  else:
    if current_state != sm.current_state:
      current_state = sm.current_state
      print("current: ", current_state)

