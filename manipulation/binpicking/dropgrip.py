from manipulation.binpicking import dropworkcell as dwc
from database import dbaccess as db

if __name__ == '__main__':
    gdb = db.GraspDB()
    dwc.checkDBrepeat(gdb)

