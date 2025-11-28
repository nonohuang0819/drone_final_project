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

     def on_exit_green(self):
         print("Exit the green")
         
     def on_enter_yellow(self):
         print("Enter the yellow")

     def on_exit_yellow(self):
         print("Exit the yellow")
         
     def on_enter_red(self):
         print("Enter the red")

     def on_exit_red(self):
         print("Exit the red")
     

sm = TrafficLightMachine()

print("saving initial statemachine png: sm1...")
img_path = "./sm1.png"
sm._graph().write_png(img_path)

print("run cycle.")
sm.send("cycle")
print("current state: ", sm.current_state)
print("current state is yellow ? using TrafficLightMachine.yellow: ", sm.current_state == TrafficLightMachine.yellow)
print("current state is yellow ? using sm.yellow: ", sm.current_state == sm.yellow)
print("now is green: ", sm.green.is_active)
print("now is yellow: ", sm.yellow.is_active)
print("now is red: ", sm.red.is_active)
print("states: ", [s.id for s in sm.states])
print("transition events: ", [t.id for t in sm.events])

print("saving initial statemachine png: sm2...")
img_path = "./sm2.png"
sm._graph().write_png(img_path)

print("run cycle")
sm.cycle()

print("saving initial statemachine png: sm3...")
img_path = "./sm3.png"
sm._graph().write_png(img_path)

print("run cycle")
sm.send('cycle')
print("now is green: ", sm.green.is_active)

print("try to change go")
sm.send("go")


