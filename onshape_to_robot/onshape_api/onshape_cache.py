"""Cache objects locally for faster exports."""
from pathlib import Path
from typing import (
    Callable,
    Tuple,
    Union,
)

from sqlalchemy import (
    BLOB,
    Column,
    Integer,
    String,
    Text,
    create_engine,
)
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.orm.exc import NoResultFound
from sqlalchemy.schema import UniqueConstraint

Base = declarative_base()


class Cache(Base):
    __tablename__ = "cache"
    __table_args__ = (UniqueConstraint("method", "key", name="unique_method_key"),)
    id = Column(Integer, primary_key=True, autoincrement=True)
    method = Column(String(255), nullable=False)
    key = Column(String(4096), nullable=False)
    text = Column(Text, nullable=True)
    blob = Column(BLOB, nullable=True)


class OnshapeCache:
    """Cache objects locally for faster exports."""

    def __init__(self, path: Path):
        """Construct a new OnshapeCache."""
        engine = create_engine(f"sqlite:///{str(path)}")
        Base.metadata.create_all(engine)
        Base.metadata.bind = engine
        self.session = sessionmaker(bind=engine)()

    def get_or_add(
        self, method: str, key: Union[str, Tuple], callback: Callable, is_string=False
    ) -> Union[str, bytes]:
        """Retrieve an object from the cache or produce it with the callback."""
        if isinstance(key, tuple):
            key = "_".join(list(key))
        try:
            query = (
                self.session.query(Cache)
                .filter(Cache.method == method, Cache.key == key)
                .one()
            )
            result = query.text if is_string else query.blob
        except NoResultFound:
            result = callback().content
            if is_string:
                if isinstance(result, bytes):
                    result = result.decode("utf-8")
                self.session.add(Cache(method=method, key=key, text=result))
            else:
                self.session.add(Cache(method=method, key=key, blob=result))
            self.session.commit()
        return result
