import sys
inPy3k = sys.version_info[0] == 3

if inPy3k:
    def force_str(s):
        if isinstance(s, bytes):
            return s.decode('utf-8')
        return str(s)

    def iteritems(d):
        return d.items()
else:
    def force_str(s):
        return str(s)

    def iteritems(d):
        return d.iteritems()
