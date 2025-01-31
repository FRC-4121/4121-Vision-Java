package frc.vision.pipeline;

import org.opencv.core.Mat;

/**
 * A queue based on a circular buffer.
 *
 * If the capacity is exceeded, the oldest elements are removed. Note that this doesn't actually implement Queue because I'm too lazy to implement all of the Collection methods
 */
public class RingBuffer {
    protected Mat[] elems;
    protected int start;
    protected int end;

    public RingBuffer(int capacity) {
        elems = new Mat[capacity + 1];
        start = 0;
        end = 0;
    }

    public int capacity() {
        return modulus() - 1;
    }
    private int modulus() {
        return elems.length;
    }
    public synchronized int size() {
        int len = end - start;
        if (len < 0) len += modulus();
        return len;
    }

    public synchronized boolean add(Mat elem) {
        if (elems[end] == null) elems[end] = new Mat();
        elem.copyTo(elems[end]);
        if (++end == modulus()) {
            end = 0;
        }
        if (start == end && ++start == modulus()) start = 0;
        return true;
    }
    public boolean offer(Mat elem) {
        return add(elem);
    }
    public synchronized Mat remove() {
        if (start == end) throw new IllegalStateException("Attempted to remove from an empty queue");
        Mat out = elems[start++];
        if (start == modulus()) start = 0;
        return out;
    }
    public synchronized Mat poll() {
        if (start == end) return null;
        Mat out = elems[start++];
        if (start == modulus()) start = 0;
        return out;
    }
    public synchronized Mat element() {
        if (start == end) throw new IllegalStateException("Attempted to get first element of an empty queue");
        return elems[start];
    }
    public synchronized Mat peek() {
        return start == end ? null : elems[start];
    }

    public void clear() {
        start = 0;
        end = 0;
    }

}
