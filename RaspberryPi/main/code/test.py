from android import AndroidLink
from android import AndroidMessage
from stm32 import STMLink
import time
link2=AndroidLink()
link2.connect()

 

to_stm=link2.recv()
print(to_stm)

