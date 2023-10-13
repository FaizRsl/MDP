from android import AndroidLink
from android import AndroidMessage
from stm32 import STMLink
import time
link2=AndroidLink()
link2.connect()

#link1= STMLink()
#link1.connect()

while True:
    request= input("1 for receive, 2 for send, 3 for end:")
    if request == "1":
        msg = link2.recv()
        #link1.send(msg)
    elif request == "2":
        cat_part=input("type your cat:")
        value_part=input("type your value:")
        msg=AndroidMessage(cat_part, value_part)
        link2.send(msg)
    else: break

link2.disconnect()





    