// automatically generated by the FlatBuffers compiler, do not modify

package motis.ppr;

import java.nio.*;
import java.lang.*;
import java.util.*;
import com.google.flatbuffers.*;

@SuppressWarnings("unused")
public final class Routes extends Table {
  public static Routes getRootAsRoutes(ByteBuffer _bb) { return getRootAsRoutes(_bb, new Routes()); }
  public static Routes getRootAsRoutes(ByteBuffer _bb, Routes obj) { _bb.order(ByteOrder.LITTLE_ENDIAN); return (obj.__init(_bb.getInt(_bb.position()) + _bb.position(), _bb)); }
  public Routes __init(int _i, ByteBuffer _bb) { bb_pos = _i; bb = _bb; return this; }

  public motis.ppr.Route routes(int j) { return routes(new motis.ppr.Route(), j); }
  public motis.ppr.Route routes(motis.ppr.Route obj, int j) { int o = __offset(4); return o != 0 ? obj.__init(__indirect(__vector(o) + j * 4), bb) : null; }
  public int routesLength() { int o = __offset(4); return o != 0 ? __vector_len(o) : 0; }

  public static int createRoutes(FlatBufferBuilder builder,
      int routesOffset) {
    builder.startObject(1);
    Routes.addRoutes(builder, routesOffset);
    return Routes.endRoutes(builder);
  }

  public static void startRoutes(FlatBufferBuilder builder) { builder.startObject(1); }
  public static void addRoutes(FlatBufferBuilder builder, int routesOffset) { builder.addOffset(0, routesOffset, 0); }
  public static int createRoutesVector(FlatBufferBuilder builder, int[] data) { builder.startVector(4, data.length, 4); for (int i = data.length - 1; i >= 0; i--) builder.addOffset(data[i]); return builder.endVector(); }
  public static void startRoutesVector(FlatBufferBuilder builder, int numElems) { builder.startVector(4, numElems, 4); }
  public static int endRoutes(FlatBufferBuilder builder) {
    int o = builder.endObject();
    return o;
  }
};

