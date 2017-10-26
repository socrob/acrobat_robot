import Pyro4

if __name__=='__main__':
    #ip = interprocess_plotter(2)
    pe = Pyro4.Proxy("PYRONAME:proxy.enabler")
    import time
    for i in range(100):
        pe.desenha(i/10, i/10+2)
        time.sleep(0.05)

    time.sleep(5)