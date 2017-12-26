from pwn import *
server = listen(bindaddr='0.0.0.0',fam='ipv4',port=1337)
# print(dir(server))
_ = server.wait_for_connection()
while True:
	try:
		print server.recvline().strip()
	except EOFError:
		break
print "All Done"