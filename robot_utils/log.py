
def msn_info(*args, **kwargs):
    print("\033[34m[INFO]\033[0m", end="")
    print(*args, **kwargs)

def msn_debug(*args, **kwargs):
    print("\033[32m[DEBUG]\033[0m", end="")
    print(*args, **kwargs)

def msn_warn(*args, **kwargs):
    print("\033[33m[WARN]\033[0m", end="")
    print(*args, **kwargs)

def msn_error(*args, **kwargs):
    print("\033[31m[ERROR]\033[0m", end="")
    print(*args, **kwargs)
