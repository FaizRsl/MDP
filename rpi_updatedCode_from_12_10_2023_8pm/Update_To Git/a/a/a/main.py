from stm32 import STMLink

link1= STMLink()
link1.connect()
while True:
    msg= input("Say something:")
    link1.send(msg)
link1.recv()