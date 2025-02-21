package frc.vision.pipeline;

import java.util.AbstractQueue;
import java.util.Iterator;
import org.opencv.core.Mat;

/**
 * A queue based on a circular buffer.
 *
 * If the capacity is exceeded, the oldest elements are removed. Note that this doesn't actually implement Queue because I'm too lazy to implement all of the Collection methods
 */
public class RingBuffer extends AbstractQueue<Mat> {
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
    @Override
    public synchronized int size() {
        int len = end - start;
        if (len < 0) len += modulus();
        return len;
    }

    @Override
    public synchronized boolean offer(Mat elem) {
        if (elems[end] == null) elems[end] = new Mat();
        elem.copyTo(elems[end]);
        if (++end == modulus()) {
            end = 0;
        }
        if (start == end && ++start == modulus()) start = 0;
        return true;
    }
    @Override
    public synchronized Mat poll() {
        if (start == end) return null;
        Mat out = elems[start++];
        if (start == modulus()) start = 0;
        return out;
    }
    @Override
    public synchronized Mat peek() {
        return start == end ? null : elems[start];
    }

    @Override
    public void clear() {
        start = 0;
        end = 0;
    }
    @Override
    public Iter iterator() {
        return new Iter();
    }
    
    public class Iter implements Iterator<Mat> {
        @Override
        public boolean hasNext() {
            return size() == 0;
        }
        @Override
        public Mat next() {
            return poll();
        }
    }
}

