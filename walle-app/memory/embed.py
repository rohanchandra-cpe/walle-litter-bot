"""Text embedders for the memory store's free-text recall.

OllamaEmbedder calls Ollama's /api/embed on the host. If Ollama can't be reached it says so
once and returns None from then on, so writes never wait on it again; the memory store then
keeps the text without an embedding (embed it later with MemoryStore.embed_pending()).
FakeEmbedder is deterministic and offline, for tests.
"""

import hashlib
import json
import math
import os
import re
import urllib.error
import urllib.request


class OllamaEmbedder:
    def __init__(self, model="nomic-embed-text", urls=(), timeout_s=5.0):
        self.model = model
        env = os.environ.get("OLLAMA_URL")
        self.urls = [u.rstrip("/") for u in ([env] if env else []) + list(urls)]
        self.timeout_s = timeout_s
        self.url = None               # the one that answered
        self.down = False             # gave up for this session
        self.error = None

    def embed(self, texts):
        """List of vectors (lists of floats) for `texts`, or None if Ollama is unreachable."""
        if self.down or not texts:
            return None
        for url in ([self.url] if self.url else self.urls):
            body = json.dumps({"model": self.model, "input": list(texts)}).encode()
            req = urllib.request.Request(url + "/api/embed", data=body,
                                         headers={"Content-Type": "application/json"})
            try:
                with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                    vecs = json.load(r)["embeddings"]
                self.url = url
                return vecs
            except (urllib.error.URLError, OSError, KeyError, ValueError) as e:
                self.error = f"{url}: {e}"
        self.down = True
        print(f"memory: no embeddings (Ollama '{self.model}' not reachable: {self.error}); "
              "text kept, recall uses keywords until embed_pending() runs")
        return None


class FakeEmbedder:
    """Bag of hashed words, normalized: texts sharing words are close. Offline, for tests."""
    model = "fake-bow-64"

    def __init__(self, dims=64, down=False):
        self.dims, self.down = dims, down

    def embed(self, texts):
        if self.down:
            return None
        out = []
        for t in texts:
            v = [0.0] * self.dims
            for w in re.findall(r"[a-z0-9_]+", t.lower()):
                v[int(hashlib.md5(w.encode()).hexdigest(), 16) % self.dims] += 1.0
            n = math.sqrt(sum(x * x for x in v)) or 1.0
            out.append([x / n for x in v])
        return out
