gazeKey = 0xA7
gazeFocus = 0x80
gazePitch = 0x80
gazeYaw = 0x80

newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
newMessage = str(newCommand) + "\r"

print newMessage