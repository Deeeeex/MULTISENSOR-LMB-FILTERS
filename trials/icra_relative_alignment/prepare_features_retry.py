"""Retry only transient public transport setup; keep frozen grid code intact."""
import time
import urllib.error
import prepare_features as original

class RetryArchive(original.Archive):
    def refresh(self):
        for attempt in range(5):
            try:return super().refresh()
            except (urllib.error.URLError,TimeoutError,ConnectionError):
                if attempt==4:raise RuntimeError(f'Public archive connection setup failed after five attempts: {self.name}') from None
                print('PUBLIC ARCHIVE CONNECTION RETRY',self.name,attempt+1,flush=True)
                time.sleep(attempt+1)

if __name__=='__main__':
    original.Archive=RetryArchive
    original.main()
