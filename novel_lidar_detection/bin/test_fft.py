#!/usr/bin/env python
from numpy import inf
import numpy as np
import matplotlib.pyplot as plt
import math
wsz = 10
wsp = 3
max_range = 10
threshold = 0.2
projection = 'polar'
# projection = None
def calculate_position_from_index(median_index):
    z = 1
    
    rad = float(median_index)/len(scan) * 2 * math.pi
    x = scan[median_index] * math.cos(rad) * max_range
    y = scan[median_index] * math.sin(rad) * max_range
    return x,y,z
def window_stack(a):
        '''
        Function from here:
            https://stackoverflow.com/questions/15722324/sliding-window-in-numpy
        Returns a sliding window over a sample, a
        params
        ------
        a : np.array(m,n)
            Sample to slide over

        returns
        -------
        np.array(m,d)
            A stack of windowed samples, where d is the window size
        np.array(m,d)
            An array of the same size that gives the indicies of the windowed data
        '''
   
        num_windows = math.floor(float(len(a) - wsz + 1)/wsp)
        indexer = np.arange(wsz)[None, :] + wsp*np.arange(num_windows)[:, None]
        indexer = indexer.astype(np.int)
        return (a[indexer], indexer)
scan =  [inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.347663402557373, 1.3766728639602661, 1.4504015445709229, 1.5102511644363403, 1.590195655822754, 1.6696714162826538, 1.7042280435562134, 1.6868311166763306, 1.6826666593551636, 1.6609225273132324, 1.666453242301941, 1.6622852087020874, 1.672603964805603, 1.6367160081863403, 1.6428707838058472, 1.6375727653503418, 1.6514006853103638, 1.6457703113555908, 1.6332951784133911, 1.6290849447250366, 1.6233259439468384, 1.6278767585754395, 1.6254122257232666, 1.617986798286438, 1.617924451828003, 1.6282823085784912, 1.6324869394302368, 1.6295865774154663, 1.627267837524414, 1.634459376335144, 1.6336662769317627, 1.633402705192566, 1.6432747840881348, 1.6454718112945557, 1.632992148399353, 1.6590304374694824, 1.6223852634429932, 1.4984194040298462, 1.3859230279922485, 1.3001779317855835, 1.2765038013458252, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.554568290710449, 2.526397228240967, 2.480729579925537, 2.421403408050537, 2.4087302684783936, 2.354471445083618, 2.306791305541992, 2.2996182441711426, 2.2602956295013428, 2.2254531383514404, 2.197808027267456, 2.202861785888672, 2.150137424468994, 2.1240780353546143, 2.1146185398101807, 2.1036360263824463, 2.0583040714263916, 2.0517914295196533, 2.0234103202819824, 2.014777660369873, 1.9835686683654785, 2.0228729248046875, 2.1095614433288574, 2.188825845718384, 2.2734830379486084, 2.383479356765747, 2.5277962684631348, 2.643869400024414, 2.8161425590515137, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.4628846645355225, 3.4259755611419678, 3.3823347091674805, 3.3629579544067383, 3.3086705207824707, 3.298358201980591, 3.269831418991089, 3.2275991439819336, 3.205437660217285, 3.1835436820983887, 3.169788360595703, 3.1359899044036865, 3.112888813018799, 3.0801634788513184, 3.0750226974487305, 3.039921760559082, 3.008681535720825, 3.015601396560669, 3.0039355754852295, 3.006441116333008, 2.9772181510925293, 2.9474971294403076, 2.9311091899871826, 2.9600977897644043, 2.922714948654175, 2.9136130809783936, 2.9161550998687744, 3.1799075603485107, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.471148729324341, 3.4513180255889893, 3.4183757305145264, 3.4084630012512207, 3.3985891342163086, 3.381134271621704, 3.406244993209839, 3.416201114654541, 3.4552536010742188, 3.4777090549468994, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.1367948055267334, 2.8905014991760254, 2.909529447555542, 2.907804012298584, 2.9292831420898438, 2.9175376892089844, 2.9285988807678223, 2.94463849067688, 2.9768128395080566, 2.9943439960479736, 3.003922700881958, 3.0053727626800537, 3.043546199798584, 3.053715467453003, 3.069887399673462, 3.0832715034484863, 3.1307120323181152, 3.134254217147827, 3.1676242351531982, 3.1804654598236084, 3.2191073894500732, 3.2588398456573486, 3.2889015674591064, 3.314922571182251, 3.348721981048584, 3.375629425048828, 3.417306661605835, inf, inf, inf, inf, inf, inf, inf, 1.2309050559997559, 1.2392876148223877, 1.223669409751892, 1.2207229137420654, 1.2128229141235352, 1.213283896446228, 1.2087122201919556, 1.2151374816894531, 1.2127318382263184, 1.2076102495193481, 1.214100956916809, 1.217924952507019, 1.2100348472595215, 1.2269697189331055, 1.2360107898712158, 1.2193683385849, 1.2245672941207886, 1.2507295608520508, 1.2397173643112183, 1.2520164251327515, 1.257615089416504, 1.2519550323486328, 1.2621957063674927, 1.2643678188323975, 1.2891024351119995, 1.2987836599349976, 1.281313180923462, 1.298504114151001, 1.2988333702087402, 1.322401523590088, 1.3133786916732788, 1.347331166267395, 1.3386945724487305, 1.354775071144104, 1.3749443292617798, 1.3837356567382812, 1.3965002298355103, 1.417687177658081, 1.4192142486572266, 1.447814702987671, 1.458389401435852, 1.467535376548767, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]
expected_scan = [inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.5500000715255737, 1.5500000715255737, 1.600000023841858, 1.649999976158142, 1.75, 1.850000023841858, 1.899999976158142, 1.850000023841858, 1.850000023841858, 1.850000023841858, 1.850000023841858, 1.850000023841858, 1.850000023841858, 1.850000023841858, 1.850000023841858, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.8000000715255737, 1.850000023841858, 1.850000023841858, 1.7000000476837158, 1.5500000715255737, 1.4500000476837158, 1.4500000476837158, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.450000047683716, 3.4000000953674316, 3.3500001430511475, 3.3500001430511475, 3.299999952316284, 3.25, 3.200000047683716, 3.200000047683716, 3.1500000953674316, 3.1500000953674316, 3.1000001430511475, 3.1000001430511475, 3.1000001430511475, 3.0, 3.0, 3.0, 3.0, 2.950000047683716, 2.950000047683716, 2.9000000953674316, 2.9000000953674316, 2.9000000953674316, 2.9000000953674316, 2.8500001430511475, 2.8500001430511475, 2.9000000953674316, 2.8500001430511475, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.299999952316284, 3.25, inf, 3.4000000953674316, 3.299999952316284, 3.25, 3.200000047683716, 3.200000047683716, 3.200000047683716, 3.200000047683716, 3.200000047683716, 3.200000047683716, 3.200000047683716, 3.25, 3.25, 3.299999952316284, 3.299999952316284, 3.3500001430511475, 3.450000047683716, 3.3500001430511475, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.4000000953674316, 3.0, 2.9000000953674316, 2.700000047683716, 2.6500000953674316, 2.6500000953674316, 2.6500000953674316, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 2.75, 2.799999952316284, 2.799999952316284, 2.8500001430511475, 2.8500001430511475, 2.8500001430511475, 2.9000000953674316, 2.9000000953674316, 2.950000047683716, 3.0, 3.0, 3.049999952316284, 3.049999952316284, 3.1000001430511475, 3.1500000953674316, 3.200000047683716, 3.200000047683716, 3.25, 3.299999952316284, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.1000001430511475, 1.75, 1.5500000715255737, 1.399999976158142, 1.2000000476837158, 1.100000023841858, 1.100000023841858, 1.149999976158142, 1.149999976158142, 1.149999976158142, 1.149999976158142, 1.149999976158142, 1.149999976158142, 1.149999976158142, 1.2000000476837158, 1.2000000476837158, 1.149999976158142, 1.2000000476837158, 1.2000000476837158, 1.2000000476837158, 1.2000000476837158, 1.25, 1.25, 1.25, 1.25, 1.25, 1.350000023841858, 1.3000000715255737, 1.3000000715255737, 1.350000023841858, 1.350000023841858, 1.350000023841858, 1.399999976158142, 1.399999976158142, 1.399999976158142, 1.350000023841858, 1.399999976158142, 1.399999976158142, 1.4500000476837158, 1.4500000476837158, 1.5, 1.5, 1.5500000715255737, 1.5500000715255737, 1.5500000715255737, 1.5500000715255737, 1.5500000715255737, 1.5500000715255737, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]

    
scan = np.array(scan)

expected_scan = np.array(expected_scan)
scan[scan==inf] = np.nan
expected_scan[expected_scan==inf] = np.nan
print('Len of scan {}'.format(len(scan)))
print('Len of Escan {}'.format(len(expected_scan)))




# scan[scan==np.nan] = np.inf
# expected_scan[expected_scan==np.nan] = np.inf
print(scan)
print(expected_scan)
scan[np.isnan(scan)] = max_range
expected_scan[np.isnan(expected_scan)] = max_range
scan = scan/max_range
expected_scan = expected_scan/max_range
ax = plt.subplot(221, projection=projection)
ax.plot(np.arange(0,2*np.pi, 2*np.pi/360), scan)

ax = plt.subplot(222, projection=projection)
ax.plot(np.arange(0,2*np.pi, 2*np.pi/360), expected_scan)
print(scan)
print(expected_scan)
best_offset = 0
best_err = np.inf
for i in range(len(scan)):
    # s = np.roll(scan, i)
    es = np.roll(expected_scan, i)
    err = np.sum((scan-es)**2)
    if err < best_err:
        best_offset = i
        best_err = err
print(best_offset)
plt.subplot(224, projection=projection)
expected_scan = np.roll(expected_scan, best_offset)
plt.plot(np.arange(0,2*np.pi, 2*np.pi/360),)

# fscan = np.fft.fft(scan)
# fescan = np.fft.fft(expected_scan)

# ax = plt.subplot(323)
# ax.plot(np.angle(fscan))

# ax = plt.subplot(324)
# ax.plot(np.angle(fescan))
# ax = plt.subplot(325)
# ax.plot(np.abs(fscan))


# ax = plt.subplot(326)
# ax.plot(np.abs(fescan))




# plt.show()
# plt.figure()

sw, sw_i = window_stack(scan)
ew, ew_i = window_stack(expected_scan)
detected_objects = np.zeros(scan.shape, dtype=np.bool)
detected_objects = np.zeros(scan.shape, dtype=np.uint8)

er =[]
for s,e,i in zip(sw, ew, sw_i):
    # er = np.correlate(s,e)
    er.append(((s-e)**2).mean())
    if er[-1] > threshold:
        detected_objects[i] = 1
er.extend([0,0])


# plt.subplot(223, projection=projection)
# plt.plot(np.arange(0,2*np.pi, 2*np.pi/360),er )

er2 = expected_scan - scan
# plt.plot(np.arange(0,2*np.pi, 2*np.pi/360),er2 )
# er2 = scan - expected_scan
er2[er2<0] = 0
kernel = np.ones(wsz) * 1.0/wsz
er2 = np.convolve(er2, kernel, mode='same')
detected_objects = er2>threshold
# Calculate center of mass of objects
detected_objects_position = []
in_object = False
pos_begin, pos_end, i = 0, 0, 0
while i < len(detected_objects):
    if detected_objects[i]:
        if not in_object:
            pos_begin = i
            in_object = True
    else:
        if in_object:
            pos_end = i
            com = math.floor((pos_begin+pos_end)/2)
            detected_objects_position.append(int(com))
            in_object = False
    i += 1
for o in detected_objects_position:
    print(calculate_position_from_index(o))
# plt.subplot(224, projection=projection)
# plt.plot(np.arange(0,2*np.pi, 2*np.pi/360),er2)

plt.subplot(223, projection=projection)
plt.plot(np.arange(0,2*np.pi, 2*np.pi/360),detected_objects)
plt.show()



