import weakref


class WeakMethod:

    def __init__(self, f):
        self.f = f.im_func
        self.c = weakref.ref(f.im_self)

    def __call__(self, *arg):
        if self.c() is None:
            raise TypeError('Method called on dead object')
        apply(self.f, (self.c(), ) + arg)
