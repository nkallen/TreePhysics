#include <metal_stdlib>
#include "Print.metal"
using namespace metal;

class Debug {
private:
    device char *buf;
    size_t idx = 0U;
    size_t maxlen;

public:
    Debug(device char *buf, size_t buflen, uint gid, uint grid_size)
    {
        int maxlen = buflen / grid_size;
        this->buf = buf + gid*maxlen;
        this->maxlen = maxlen;
    }

    thread Debug & operator <<(float x)
    {
        unsigned int flags, width, precision;

        flags = 0U;
        precision = 0U;
        width = 0U;

        idx = _ftoa(_out_buffer, buf, idx, maxlen, x, precision, width, flags);
        return *this;
    }

    thread Debug & operator <<(int x)
    {
        unsigned int flags = 0U, base = 10U, precision = 0U, width = 0U;

        idx = _ntoa_int(_out_buffer, buf, idx, maxlen, (unsigned int)(x > 0 ? x : 0 - x), x < 0, base, precision, width, flags);
        return *this;
    }

    thread Debug & operator <<(unsigned int x)
    {
        unsigned int flags = 0U, base = 10U, precision = 0U, width = 0U;

        idx = _ntoa_int(_out_buffer, buf, idx, maxlen, (unsigned int)(x > 0 ? x : 0 - x), false, base, precision, width, flags);
        return *this;
    }

    thread Debug & operator <<(half x)
    {
        unsigned int flags, width, precision;

        flags = 0U;
        precision = 0U;
        width = 0U;

        idx = _ftoa(_out_buffer, buf, idx, maxlen, x, precision, width, flags);
        return *this;
    }

    thread Debug & operator <<(const constant char * x)
    {
        while (*x) {
            _out_buffer(*x, buf, idx++, maxlen);
            x++;
        }

        return *this;
    }

    template <class T, size_t count>
    thread Debug & operator <<(vec<T, count> a)
    {
        *this << "(";
        for (size_t i = 0; i < count; i++) {
            *this << a[i];
            *this << (i < count - 1 ? "," : ")");
        }
        return *this;
    }

    template <class T>
    thread Debug & operator <<(matrix<T, 3, 3> a)
    {
        *this << "(";
        for (size_t i = 0; i < 3; i++) {
            *this << a[i];
        }
        *this << ")";
        return *this;
    }
};
