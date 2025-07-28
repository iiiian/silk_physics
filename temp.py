a = -0.486352712
b = 1
c = -0.515330732
d = 0.0014015846
tol = 0.1


def eval(x):
    return a*x*x*x + b*x*x + c*x + d


def eval_d(x):
    return x * (x * 3 * a + 2 * b) + c


def newton(x):
    x_next = x + tol
    x_next_eval = eval(x_next)
    cnt = 0
    while x_next_eval > 0:
        x = x - eval(x) / eval_d(x)
        x_next = x + tol
        x_next_eval = eval(x_next)

        print(f"Iter {cnt}:\n x {x}\n x_next {x_next}\n x_next_eval {
              x_next_eval}\n eval_x {eval(x)}\n eval_x_d {eval_d(x)}\n\n")
        cnt += 1
        if (cnt > 10):
            break
    return x


def bisect(left, right):
    for i in range(20):
        m = 0.5*(left+right)
        print(f"eval m {eval(m)}")
        if eval(m) > 0:
            left = m
        else:
            right = m
        print(f"l {left}, m {m}, r {right}")
    return left


print(bisect(0, 1))
