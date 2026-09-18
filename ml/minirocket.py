"""MiniRocket for multichannel series, in numpy.

Dempster, Schmidt & Webb, "MiniRocket: A Very Fast (Almost) Deterministic
Transform for Time Series Classification", KDD 2021. Written from the paper
and its reference code so the rig needs nothing beyond scikit-learn; aeon and
sktime ship the reference implementation if one is wanted for comparison.

The transform turns each series into ~10,000 numbers, none of them learned:

  kernels    84 fixed patterns, nine points long, weight -1 everywhere except
             +2 at three positions - one per way of choosing those three.
  dilations  each kernel is also read with gaps between its points, so the
             same pattern matches a sharp feature over a few samples and a
             slow one across the whole series.
  channels   each kernel/dilation pair reads a random subset of channels and
             sums them, so relationships between chambers are picked up.
  biases     thresholds taken from quantiles of that kernel's response on a
             training example.
  feature    the proportion of positions where the response exceeds the
             threshold - how much of the series looks like that pattern.

A linear classifier on those numbers does all of the learning.

One deliberate departure: channels are standardised before the transform.
The reference leaves series as they are, which is fine for one channel, but
summing channels in different units - pascals and metres here - would let the
pressure swamp the indentation entirely.

Saved models reference this module by name, so load them with ml/ on the path.
"""

import itertools

import numpy as np
from sklearn.base import BaseEstimator, TransformerMixin

KERNEL_LENGTH = 9
# Lexicographic, as in the reference, so feature order matches it.
INDICES = np.array(list(itertools.combinations(range(KERNEL_LENGTH), 3)))


def fit_dilations(length, num_features, max_dilations_per_kernel):
    """Dilations spaced exponentially up to the longest that fits the series,
    and how many features each one gets."""
    per_kernel = num_features // len(INDICES)
    true_max = min(per_kernel, max_dilations_per_kernel)
    multiplier = per_kernel / true_max
    max_exponent = np.log2((length - 1) / (KERNEL_LENGTH - 1))
    dilations, counts = np.unique(
        np.logspace(0, max_exponent, true_max, base=2).astype(np.int32),
        return_counts=True)
    per_dilation = (counts * multiplier).astype(np.int32)
    remainder = per_kernel - per_dilation.sum()
    i = 0
    while remainder > 0:
        per_dilation[i] += 1
        remainder -= 1
        i = (i + 1) % len(per_dilation)
    return dilations, per_dilation


def quantiles(n):
    """Low-discrepancy quantiles from the golden ratio, as in the reference."""
    return np.array([(i * ((np.sqrt(5) + 1) / 2)) % 1 for i in range(1, n + 1)])


def shifted(X, dilation):
    """The nine dilated taps under each position of every series.

    out[k, ..., t] = X[..., t + (k - 4) * dilation], zero past either end -
    the zero padding of a 'same' convolution.
    """
    length = X.shape[-1]
    out = np.zeros((KERNEL_LENGTH,) + X.shape)
    for k in range(KERNEL_LENGTH):
        s = (k - KERNEL_LENGTH // 2) * dilation
        if 0 <= s < length:
            out[k, ..., :length - s] = X[..., s:]
        elif -length < s < 0:
            out[k, ..., -s:] = X[..., :length + s]
    return out


def responses(S, kernel):
    """Convolution with one kernel, from the nine shifted copies.

    Weights are -1 at all nine taps and +2 at three, so the response is minus
    the sum of all nine plus three times the three chosen ones.
    """
    a, b, c = kernel
    return -S.sum(axis=0) + 3.0 * (S[a] + S[b] + S[c])


class MiniRocket(BaseEstimator, TransformerMixin):
    """Takes rows of flattened (channels x points) series, as the classifier
    builds them, and returns MiniRocket features."""

    def __init__(self, n_channels=1, num_features=10000,
                 max_dilations_per_kernel=32, random_state=0):
        self.n_channels = n_channels
        self.num_features = num_features
        self.max_dilations_per_kernel = max_dilations_per_kernel
        self.random_state = random_state

    def _series(self, X):
        X = np.asarray(X, dtype=float)
        return X.reshape(len(X), self.n_channels, -1)

    def fit(self, X, y=None):
        X = self._series(X)
        n, channels, length = X.shape
        if length < KERNEL_LENGTH:
            raise ValueError("MiniRocket needs at least %d points per channel, "
                             "got %d - raise --points." % (KERNEL_LENGTH, length))
        rng = np.random.RandomState(self.random_state)

        self.mean_ = X.mean(axis=(0, 2))[:, None]
        self.std_ = np.maximum(X.std(axis=(0, 2)), 1e-12)[:, None]
        X = (X - self.mean_) / self.std_

        self.dilations_, self.per_dilation_ = fit_dilations(
            length, self.num_features, self.max_dilations_per_kernel)
        q = quantiles(len(INDICES) * int(self.per_dilation_.sum()))
        combos = len(INDICES) * len(self.dilations_)
        # 1 up to min(channels, 9) channels per combination, skewed toward few.
        counts = (2 ** rng.uniform(0, np.log2(min(channels, 9) + 1), combos)).astype(int)
        self.channels_ = [rng.choice(channels, k, replace=False) for k in counts]

        self.biases_ = []
        feature = combo = 0
        for dilation, count in zip(self.dilations_, self.per_dilation_):
            for kernel in INDICES:
                example = X[rng.randint(n)][self.channels_[combo]]
                response = responses(shifted(example, dilation), kernel).sum(axis=0)
                self.biases_.append(np.quantile(response, q[feature:feature + count]))
                feature += count
                combo += 1
        return self

    def transform(self, X):
        X = (self._series(X) - self.mean_) / self.std_
        out = np.empty((len(X), sum(len(b) for b in self.biases_)))
        column = combo = 0
        for d_index, dilation in enumerate(self.dilations_):
            S_all = shifted(X, dilation)
            padding = ((KERNEL_LENGTH - 1) * dilation) // 2
            for k_index, kernel in enumerate(INDICES):
                S = S_all[:, :, self.channels_[combo], :]
                response = responses(S, kernel).sum(axis=1)
                # Alternate pairs are pooled over the whole series or over only
                # the part not reaching into the padding, as in the reference.
                if (d_index + k_index) % 2 == 1 and padding > 0:
                    response = response[:, padding:-padding]
                biases = self.biases_[combo]
                out[:, column:column + len(biases)] = \
                    (response[:, :, None] > biases).mean(axis=1)
                column += len(biases)
                combo += 1
        return out
