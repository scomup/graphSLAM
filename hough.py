#!/usr/bin/python
# coding: UTF-8
import numpy as np
from math import *
from scipy import signal

def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial
    
    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')


def hough(data):
    theta = np.linspace(-np.pi / 2, np.pi / 2, 180)

    ctheta = np.cos(theta)
    stheta = np.sin(theta)

    nthetas = theta.shape[0]

    max_distance = 200
    offset = max_distance / 2
    accum = np.zeros((max_distance, nthetas), dtype=np.uint32)

    """
    for i in range(data.shape[0]):
        x,y  = data[i,:]
        for j in range(nthetas):
            accum_idx = int(round( (ctheta[j] * x + stheta[j] * y)*10 )) + offset
            #print accum_idx
            accum[accum_idx, j] += 1

    """

    
    for i in range(data.shape[0]):
        x,y  = data[i,:]
        accum_idx = np.round( (ctheta * x + stheta * y)*10 ) + offset
        accum_idx = accum_idx.astype(int)
        #print accum_idx
        accum[accum_idx, np.arange(nthetas)] += 1
    

    """
    x  = data[:,0]
    y  = data[:,1]
    x  = x.reshape((1,x.size))
    y  = y.reshape((1,y.size))
    ctheta = ctheta.reshape((ctheta.size,1))
    stheta = stheta.reshape((stheta.size,1))

    #return accum

    accum_idx = np.round( np.dot(ctheta,x) + np.dot(stheta, y)*10) + offset
    accum_idx = accum_idx.astype(int)
    accum[accum_idx.T.ravel(), np.tile(np.arange(nthetas),data.shape[0])] += 1
    """
    return accum
    
def check_loop_candidate(data):
    
    accum = hough(data)
    max_data =  np.amax(accum, axis=0)
    #max_data = max_data[max_data > data.shape[0] * 0.3]
    #best_idx = np.argsort(max_data)[::-1]
    #best_idx = best_idx[(best_idx > 60) * (best_idx < 120)] 
    #if best_angle.size < 2:
    #    return False
    #window = signal.general_gaussian(51, p=0.5, sig=20)
    #filtered = signal.fftconvolve(window, max_data)
    #filtered = (np.average(data) / np.average(filtered)) * filtered

    #peakind = signal.find_peaks_cwt(filtered,np.arange(1,20))
    max_data = savitzky_golay(max_data, 31, 3)
    peakind = signal.argrelmax(max_data, order=10)[0]
    peakind = peakind[(peakind > 60) * (peakind < 120)] 
    A = np.any(max_data[peakind] > data.shape[0] * 0.1)

    #print peakind

    #print peakind[0].size
    #A = max_data[best_angle[0]] > data.shape[0] * 0.3
    #B = max_data[best_angle[1]] > data.shape[0] * 0.3

    #C = abs(best_angle[0] - best_angle[1]) > 50
    """
    import matplotlib.pyplot as plt
    plt.imshow(accum)
    plt.show()
    """

    """
    import matplotlib.pyplot as plt
    plt.cla()
    plt.plot(max_data)
    plt.plot(peakind, max_data[peakind], 'ro')
    plt.plot(np.ones(max_data.size)*data.shape[0] * 0.1)
    #for x0 in peakind:
    #    plt.plot(x0, max_data[x0], 'ro')
    plt.pause(.01)
    """
    return A
    #plt.show()
    
    




if __name__ == "__main__":
    
    data = np.load("scan1.npy")
    check_loop_candidate(data)
    """
    import matplotlib.pyplot as plt
    plt.imshow(accum)
    plt.show()
    """
    
