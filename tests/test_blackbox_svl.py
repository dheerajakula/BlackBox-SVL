from blackbox_svl import __version__


def test_version():
    assert __version__ == '0.1.0'

def test_run_test():
    x = [i for i in range(0,100,1)]
    y = [i for i in range(0,100,1)]
    
    # svl_simulator.run_test(13.2160, 100.5596, 0,x,y,10, False)