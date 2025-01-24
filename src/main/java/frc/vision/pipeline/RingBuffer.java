package frc.vision.pipeline;

import java.util.ArrayList;

/**
 * A queue based on a circular buffer.
 *
 * If the capacity is exceeded, the oldest elements are removed. Note that this doesn't actually implement Queue because I'm too lazy to implement all of the Collection methods
 */
public class RingBuffer<E> {
    protected ArrayList<E> elems;
    protected int cap;
    protected int start;
    protected int end;

    public RingBuffer(int capacity) {
        elems = new ArrayList<>(capacity + 1);
        cap = capacity + 1;
        start = 0;
        end = 0;
    }

    public int capacity() {
        return cap;
    }
    private int modulus() {
        return capacity() + 1;
    }
    public synchronized int size() {
        int len = end - start;
        if (len < 0) len += modulus();
        return len;
    }

    public synchronized boolean add(E elem) {
        if (end == elems.size()) elems.add(elem);
        else elems.set(end, elem);
        if (++end == modulus()) {
            end = 0;
        }
        if (start == end && ++start == modulus()) start = 0;
        return true;
    }
    public boolean offer(E elem) {
        return add(elem);
    }
    public synchronized E remove() {
        if (start == end) throw new IllegalStateException("Attempted to remove from an empty queue");
        E out = elems.get(start++);
        if (start == modulus()) start = 0;
        return out;
    }
    public synchronized E poll() {
        if (start == end) return null;
        E out = elems.get(start++);
        if (start == modulus()) start = 0;
        return out;
    }
    public synchronized E element() {
        if (start == end) throw new IllegalStateException("Attempted to get first element of an empty queue");
        return elems.get(start);
    }
    public synchronized E peek() {
        return start == end ? null : elems.get(start);
    }

    public void clear() {
        start = 0;
        end = 0;
    }

}
