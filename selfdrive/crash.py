"""Install exception handler for process crash."""
import os
import sys
import threading
import capnp
from selfdrive.version import version, dirty

from selfdrive.swaglog import cloudlog
from common.hardware import PC

if os.getenv("NOLOG") or os.getenv("NOCRASH") or PC:
  def capture_exception(*args, **kwargs):
    pass

  def bind_user(**kwargs):
    pass

  def bind_extra(**kwargs):
    pass

  def install():
    pass
else:
  from raven import Client
  from raven.transport.http import HTTPTransport
  client = Client('https://ee3dca66da104ef388e010fcefbd06c6:df79d17e3a0743c387d4cbf05932abde@o484202.ingest.sentry.io/5537090',
                  install_sys_hook=False, transport=HTTPTransport, release=version, tags={'dirty': dirty})

  def capture_exception(*args, **kwargs):
    exc_info = sys.exc_info()
    if not exc_info[0] is capnp.lib.capnp.KjException:
      client.captureException(*args, **kwargs)
    cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  def bind_user(**kwargs):
    client.user_context(kwargs)

  def bind_extra(**kwargs):
    client.extra_context(kwargs)

  def install():
    """
    Workaround for `sys.excepthook` thread bug from:
    http://bugs.python.org/issue1230540
    Call once from the main thread before creating any threads.
    Source: https://stackoverflow.com/a/31622038
    """
    # installs a sys.excepthook
    __excepthook__ = sys.excepthook

    def handle_exception(*exc_info):
      if exc_info[0] not in (KeyboardInterrupt, SystemExit):
        capture_exception()
      __excepthook__(*exc_info)
    sys.excepthook = handle_exception

    init_original = threading.Thread.__init__

    def init(self, *args, **kwargs):
      init_original(self, *args, **kwargs)
      run_original = self.run

      def run_with_except_hook(*args2, **kwargs2):
        try:
          run_original(*args2, **kwargs2)
        except Exception:
          sys.excepthook(*sys.exc_info())

      self.run = run_with_except_hook

    threading.Thread.__init__ = init
