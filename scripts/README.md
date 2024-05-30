## Generate Shared Object for mother_interface.py
Move to scripts directory

```
git clone git@github.com:cmcqueen/cobs-c.git -d /tmp/cobs
gcc /tmp/cobs/cobs.c -shared -o libcobs.so.2.0.0
```
