def fletcher32(data, length):
    w_len = length
    c0 = 0
    c1 = 0
    x = 0

    while w_len >= 360:
        for i in range (360):
            c0 = c0 + ord(data[x])
            c1 = c1 + c0
            x = x + 1
        c0 = c0 % 65535
        c1 = c1 % 65535
        w_len = w_len - 360
    
    for i in range (w_len):
       c0 = c0 + ord(data[x])
       c1 = c1 + c0
       x = x + 1
    c0 = c0 % 65535
    c1 = c1 % 65535
    return (c1 << 16 | c0)


d1 = "a123b456c789"
l1 = len(d1) 
f1 = fletcher32(d1, l1)

d2 = "abcdef"
l2 = len(d2)
f2 = fletcher32(d2, l2)

d3 = "Hello World..."
l3 = len(d3)
f3 = fletcher32(d3, l3)

print("{:>15} -> {:08X}".format(d1, f1))
print("{:>15} -> {:08X}".format(d2, f2))
print("{:>15} -> {:08X}".format(d3, f3))
