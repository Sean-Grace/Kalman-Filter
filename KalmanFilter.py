def KalmanFilter(y,Pp,xp,Qt,F,H,R):
    ht = H.transpose()
    Ft = F.transpose()
    I = np.identity(len(F))
    x_r = xp
    i=0
    for i in range (0,len(y.T)):
        Kn = Pp*ht*((H*Pp*ht + R)**(-1))
        x0 = xp + Kn*(y[i]- xp[0,0])
        P0 = (I - Kn*H)*Pp
        #Update
        xp = F*x0
        Pp = F*P0*Ft + Qt
        x_r = np.append(x_r,xp,1)
        Kr = np.append(Kn)
        i+=1
    return x_r

def plot_kf(y ,Pp ,xp ,Qt ,F ,H ,R ,title='' ,xlab='' ,ylab=''):
    ht = H.transpose()
    Ft = F.transpose()
    I = np.identity(len(F))
    x_r = xp
    i=0
    for i in range (0,len(y.T)):
        Kn = Pp*ht*((H*Pp*ht + R)**(-1))
        x0 = xp + Kn*(y[i]- xp[0,0])
        P0 = (I - Kn*H)*Pp
        #Update
        xp = F*x0
        Pp = F*P0*Ft + Qt
        x_r = np.append(x_r,xp,1)
        Kr = np.append(Kn)
        i+=1
    import matplotlib.pyplot as plt 
    xg = np.linspace(1,len(y),len(y))
    xf = np.linspace(0,len(y),len(y)+1)
    plt.plot(xg,y1,'r--',label='Observed')
    plt.plot(xf, x1.T,'b+',label='Filtered')
    plt.plot(xg,xg**0,label='Actual')
    plt.xlabel(xlab)
    plt.ylabel(ylab)
    plt.legend()
    plt.title(title,size=16)
    plt.show()
    
def EKF(y,P0,x0,Qt,R,Jacobian, Eval):
    I = np.identity(len(P0))
    xr = x0
    n=len(y.T)
    i=0
    for i in range (0,n):
        #Update
        F = Jacobian(x0[0,0],x0[1,0],x0[2,0])
        xp = x0 + Eval(x0[0,0],x0[1,0],x0[2,0])*0.01
        #Prediction
        Pp = F*P0*F.T + Qt
        H = np.matrix([F[0,0],0,0,0,0])
        S=H*Pp*H.T + R #Technicality of matrix multiplication
        Kn = Pp*H.T*(1/S[0,0])
        x0 = xp + Kn*(y[0,i]- xp[0,0])
        P0 = (I - Kn*H)*Pp

        #Save
        x1=np.array([[x0[0,0]],[x0[1,0]],[x0[2,0]]])
        xr = np.append(xr,x1,1)
        i+=1
    return(xr)



