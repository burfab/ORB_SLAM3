import numpy as np

arr = [0.0080019, 0.0019920, 0.9999660, 0.002,
        -0.9999660, 0.00200080, 0.0079979, 0.023,
        -0.0019920, -0.9999960, 0.0020080,   -0.000,
        0.000000, 0.000000, 0.000000, 1.00000]

arr = [0.999999, 0.000924, 0.000770, 0.023000,
        -0.000925, 1.000000, 0.000255, -0.000220,
        -0.000770, -0.000256, 1.000000, -0.002000,
        0.000000, 0.000000, 0.000000, 1.000000]

arr = [0.999626 ,0.025546 ,0.009792, -0.002000,
        -0.025588, 0.999664 ,0.004221,-0.023000,
        -0.009681, -0.004470 ,0.999943,-0.002000,
        0.000000, 0.000000 ,0.000000,1.000000]


arr = np.array(arr)

arr = arr.reshape((4,4))

print(arr)
print("inv: ", np.linalg.inv(arr))


print("pretty: ")
inv = np.linalg.inv(arr)

s = "["
for i in range(arr.shape[0]):
    for j in range(arr.shape[1]):
        s+= f"{inv[i,j]}, "

    s+="\n"

s = s[0:-3]
s += "]"
print(s)
