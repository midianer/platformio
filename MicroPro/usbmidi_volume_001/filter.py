import numpy as np
import matplotlib.pyplot as plt
import pylab

##xk = 5
##a = 1
##
##for i in range(1, 10):
##    xk = a*0.1 + 0.9*xk;
##    print(xk);     
    
input = 1.0 * 256;
output = 0.0;
Qcoeff = 1024;
Icoeff = 0.20 * Qcoeff;
res = [];
res1 = [];
amp = [];
ak1 = [0,0,0];
bk1 = [1,0,0];

#// Neue Berechnung
for i in range(1, 50):
  output = (int)((Icoeff * input) + (((Qcoeff - Icoeff) * output) / Qcoeff));
  ##output = int(output);
  print(output/Qcoeff);
  res1.append(output/Qcoeff)

plt.plot(res1)
plt.ylabel('Einschwingen 1')
plt.show()

##
###define Q15_ONE (1<<15) // 32768
###define Q14_HALF (1<<14) // 16384 (für Rundung im Q30-Format)
##
##int16_t input_buffer[2]; // x[n-1], x[n]
##int16_t output_buffer[2]; // y[n-1], y[n]
##int32_t alpha_Q15; // Filterkoeffizient
##int32_t y_n; // Akkumulator (Q30)
##
##// Initialisierung:
##alpha_Q15 = (int16_t)(alpha_float * Q15_ONE); // Float zu Q15 konvertieren
##
##// Filterfunktion (pro Sample):
##y_n = (int32_t)input_buffer[1] * alpha_Q15; // x[n] * alpha (Q30)
##y_n += (int32_t)output_buffer[1] * ((Q15_ONE - alpha_Q15) << 15); // y[n-1] * (1-alpha) (Q30)
##

def get_coeff(a1, b1, l):
    al0 = 1 / (1 + a1 * l + b1 * l * l)
    be0 = 2 * (1 - b1 * l * l) / (1 + a1 * l + b1 * l * l)
    be1 = (1 - a1 * l + b1 * l * l) / (1 + a1 * l + b1 * l * l)
    return al0, be0, be1

#Bessel:
a1 = 1.3397
b1 = 0.4889
a2 = 0.7743
b2 = 0.3890

fg = 10
fs = 1000

l = 1 / np.tan(3.14159*fg/fs)
print(l)
#al00, be01, be02
#al10, be11, be12

al00, be01, be02 = get_coeff(a1, b1, l)
ak1[0], bk1[1], bk1[2] = get_coeff(a1, b1, l)
al10, be11, be12 = get_coeff(a2, b2, l)
print(al00, be01, be02)
print(al10, be11, be12)

z0 = z1 = z2 = 0
uz = 1
yn=0

for i in range(1, 100):
    yn = al00 * uz + z0
    z1 = z2 - (be01 *  yn)
    z2 = - (be02 *  yn)
    z0 = z1
    z1 = z2
    print(yn, z0, z1, z2)
    print("{:4.4} {:4.4}".format(yn, z0))
    res.append(yn)

plt.plot(res)
plt.ylabel('Einschwingen')
plt.show()

step = 0.0001

for f in np.arange(step, 0.5, step):
  sumacos = 0
  sumasin = 0
  sumbcos = 0
  sumbsin = 0
  for i in range(0,3):
    sumacos += ak1[i] * np.cos(2 * np.pi * i * f) 
    sumasin += ak1[i] * np.sin(2 * np.pi * i * f)
    sumbcos += bk1[i] * np.cos(2 * np.pi * i * f) 
    sumbsin += bk1[i] * np.sin(2 * np.pi * i * f)
  #print(np.sqrt((sumacos*sumacos +sumasin*sumasin) / (sumbcos*sumbcos + sumbsin*sumbsin)))
  #print(f)
  amp.append(np.sqrt((sumacos*sumacos +sumasin*sumasin) / (sumbcos*sumbcos + sumbsin*sumbsin)))
  #np.append(amp, np.sqrt((sumacos*sumacos +sumasin*sumasin) / (sumbcos*sumbcos + sumbsin*sumbsin)))
  #amp.append(np.log10(np.sqrt((sumacos*sumacos +sumasin*sumasin) / (sumbcos*sumbcos + sumbsin*sumbsin))))


npamp = np.array(amp);
xx = np.arange(step, 0.5, step)
ff = np.arange(step, 0.5, step)

print(f.dtype)
plt.plot(xx, amp)
plt.xscale('log')
plt.yscale('log')
plt.grid(True)
plt.xlabel('Fs/Fg')
plt.ylabel('Amplification')
plt.xlim([step, 0.5])
plt.ylim([0.0001, 1])


fig, ax = plt.subplots()
ax.set_xlabel('Fs/Fg')
ax.set_ylabel('Ampl')
ax.set_xscale("log")
ax.set_yscale("log")
ax.plot(ff, amp)
ax.set_xlim([step, 0.5])
ax.set_ylim([0.0001, 1])
ax.grid()

plt.show()
