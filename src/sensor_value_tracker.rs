use num_traits::Float;

/// Implements exponential weighted moving average of sensor readings,
/// including exponentially fading minimum and maximum
pub struct SensorValueTracker<T> {
    /// recent minimum value (not global minimum)
    local_min: T,
    /// recent maximum value (not global maximum)
    local_max: T,
    /// exponentially weighted moving average
    average: T,
    /// weighting factor-- bigger alpha causes faster fade of old values
    alpha: T,
}

impl<T> SensorValueTracker<T>
where
    T: Float,
{
    pub fn new(alpha: T) -> Self {
        Self {
            local_min: T::nan(),
            local_max: T::nan(),
            average: T::nan(),
            alpha: alpha,
        }
    }

    pub fn average(&self) -> T {
        self.average
    }

    pub fn normalize(&self, val: T) -> T {
        val / self.range()
    }

    pub fn range(&self) -> T {
        let mut range = self.local_max - self.local_min;
        if range == T::zero() {
            range = T::one();
        }

        if range < T::zero() {
            -range
        } else {
            range
        }
    }

    pub fn update(&mut self, new_value: T) -> T {
        //seed the EMWA with the initial value
        if self.local_min.is_nan() {
            self.local_min = new_value;
        }
        if self.local_max.is_nan() {
            self.local_max = new_value;
        }
        if self.average.is_nan() {
            self.average = new_value;
        }

        self.average =
            (self.alpha * new_value) + (T::one() - self.alpha) * self.average;

        // extrema fade toward average
        if new_value > self.local_max {
            self.local_max = new_value;
        } else if new_value > self.average {
            self.local_max = (self.alpha * new_value)
                + (T::one() - self.alpha) * self.local_max;
        }
        if new_value < self.local_min {
            self.local_min = new_value;
        } else if new_value < self.average {
            self.local_min = (self.alpha * new_value)
                + (T::one() - self.alpha) * self.local_min;
        }

        self.average
    }
}
