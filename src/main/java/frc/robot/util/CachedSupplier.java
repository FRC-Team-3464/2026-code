package frc.robot.util;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Caches a computed value until invalidated.
 *
 * <p>Intended for expensive computations that do not change frequently, such as vision pose
 * estimates or derived calculations. The cached value is recomputed only after {@link
 * #invalidate()} is called.
 *
 * <p><b>Thread-safe:</b> All public methods are {@code synchronized} to allow safe use across
 * threads.
 *
 * <p>Usage example:
 *
 * <pre>{@code
 * CachedSupplier<Double> cachedValue = new CachedSupplier<>(() -> computeExpensiveValue());
 *
 * double value = cachedValue.get(); // computed once
 * cachedValue.invalidate(); // forces recomputation on next get()
 * }</pre>
 *
 * @author Maxwell Morgan
 */
public class CachedSupplier<T> implements Supplier<T> {
  private static Set<CachedSupplier<?>> instances = new HashSet<>();

  private final Supplier<T> supplier;
  private T value;
  private boolean valid = false;

  /**
   * Constructs a new {@code CachedSupplier}.
   *
   * @param supplier the computation to cache
   * @param manager the CacheManager to manage invalidation
   */
  public CachedSupplier(Supplier<T> supplier) {
    this.supplier = supplier;
    this.register();
  }

  /**
   * Returns the cached value. If the cache is invalid, recomputes the value using the underlying
   * supplier.
   *
   * <p>If the underlying supplier throws an exception, the cache remains invalid.
   *
   * @return the cached or newly computed value
   */
  @Override
  public synchronized T get() {
    if (!valid) {
      value = supplier.get();
      valid = true;
    }

    return value;
  }

  /**
   * Checks whether the cached value is currently valid.
   *
   * @return true if the cached value is valid, false otherwise
   */
  public synchronized boolean isValid() {
    return valid;
  }

  /**
   * Invalidates the cached value. The next call to {@link #get()} will recompute the value using
   * the supplier.
   */
  public synchronized void invalidate() {
    valid = false;
    value = null;
  }

  /** Invalidates all instances of {@code CachedSupplier}. Usually */
  public static synchronized void invalidateAll() {
    for (CachedSupplier<?> obj : instances) {
      obj.invalidate();
    }
  }

  /** Adds the current {@code CachedSupplier} to the static {@code instances} container. */
  private synchronized void register() {
    instances.add(this);
  }
}
