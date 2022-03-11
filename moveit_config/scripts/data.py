import numpy as np
data=np.loadtxt('date.txt')
l=len(data[:,0])
v=np.delete(data,-1,axis=0)
a=np.delete(v,-1,axis=0)

for i in range(l-1) :
    v[i][1]=(data[i+1][1]-data[i][1])/(data[i+1][0]-data[i][0])
    
for i in range(l-2) :
    a[i][1]=(v[i+1][1]-v[i][1])/(v[i+1][0]-v[i][0])
            
print(v)    
print(a)
     
