#include "motis/path/path.h"

#include <memory>
#include <regex>

#include "geo/polyline.h"
#include "geo/simplify_mask.h"
#include "geo/tile.h"

#include "utl/to_vec.h"
#include "utl/verify.h"

#include "tiles/fixed/convert.h"
#include "tiles/fixed/fixed_geometry.h"
#include "tiles/fixed/io/deserialize.h"
#include "tiles/get_tile.h"
#include "tiles/parse_tile_url.h"

#include "motis/core/common/logging.h"
#include "motis/core/access/realtime_access.h"
#include "motis/core/access/trip_access.h"
#include "motis/core/access/trip_iterator.h"
#include "motis/core/access/trip_section.h"
#include "motis/core/conv/station_conv.h"
#include "motis/core/conv/timestamp_reason_conv.h"
#include "motis/core/conv/transport_conv.h"
#include "motis/core/conv/trip_conv.h"
#include "motis/module/context/get_schedule.h"
#include "motis/module/context/motis_call.h"

#include "motis/path/constants.h"
#include "motis/path/path_database.h"
#include "motis/path/path_index.h"
#include "motis/path/prepare/source_spec.h"

#include "motis/path/fbs/InternalDbSequence_generated.h"

using namespace flatbuffers;
using namespace motis::module;
using namespace motis::access;
using namespace motis::logging;

namespace motis::path {

struct path::data {
  msg_ptr get_response(std::string const& index,
                       int const zoom_level = -1) const {
    message_creator mc;
    mc.create_and_finish(MsgContent_PathSeqResponse,
                         reconstruct_sequence(mc, index, zoom_level).Union());
    return make_msg(mc);
  }

  Offset<PathSeqResponse> reconstruct_sequence(
      message_creator& mc, std::string index, int const zoom_level = -1) const {

    auto const buf = db_->get(index);
    auto const* ptr = flatbuffers::GetRoot<InternalDbSequence>(buf.data());

    auto const station_ids = utl::to_vec(
        *ptr->station_ids(),
        [&mc](auto const& e) { return mc.CreateString(e->c_str()); });
    auto const classes = utl::to_vec(*ptr->classes());

    auto txn = db_->db_handle_->make_txn();
    auto features_dbi = db_->db_handle_->features_dbi(txn);
    auto features_cursor = lmdb::cursor{txn, features_dbi};

    std::vector<Offset<Segment>> fbs_segments;
    for (auto const* segment : *ptr->segments()) {
      utl::verify(segment->hints_rle()->size() % 2 == 0, "invalid rle");
      size_t feature_idx = 0;

      std::vector<double> coordinates;
      std::vector<int64_t> osm_node_ids;
      auto const append_coord = [&](auto const& p) {
        auto const ll = tiles::fixed_to_latlng(p);
        if (coordinates.empty() ||
            (coordinates[coordinates.size() - 2] != ll.lat_ &&
             coordinates[coordinates.size() - 1] != ll.lng_)) {
          coordinates.push_back(ll.lat_);
          coordinates.push_back(ll.lng_);
          osm_node_ids.push_back(-1);
        }
      };

      // TODO this is the most naive way to implement this query!
      for (auto i = 0ULL; i < segment->hints_rle()->size(); i += 2) {
        auto const tile = tiles::tile_key_to_tile(segment->hints_rle()->Get(i));

        tiles::pack_records_foreach(
            features_cursor, tile, [&](auto, auto pack_record) {
              for (auto j = 0ULL; j < segment->hints_rle()->Get(i + 1); ++j) {
                tiles::unpack_features(
                    db_->pack_handle_->get(pack_record),
                    [&](auto const& feature_str) {
                      auto const feature = deserialize_feature(
                          feature_str, render_ctx_.metadata_decoder_,
                          tiles::fixed_box{
                              {tiles::kInvalidBoxHint, tiles::kInvalidBoxHint},
                              {tiles::kInvalidBoxHint, tiles::kInvalidBoxHint}},
                          zoom_level < 0 ? tiles::kInvalidZoomLevel
                                         : zoom_level);

                      utl::verify(feature.has_value(), "deserialize failed");

                      if (feature->id_ !=
                          std::abs(segment->features()->Get(feature_idx))) {
                        return;
                      }

                      auto const& l =
                          mpark::get<tiles::fixed_polyline>(feature->geometry_);
                      utl::verify(l.size() == 1 && l.front().size() > 1,
                                  "invalid line geometry");

                      if (feature_idx < 0) {
                        std::for_each(std::rbegin(l.front()),
                                      std::rend(l.front()), append_coord);
                      } else {
                        std::for_each(std::begin(l.front()),
                                      std::end(l.front()), append_coord);
                      }
                    });
                ++feature_idx;
              }
            });
      }

      utl::verify(coordinates.size() >= 2, "empty coordinates");
      if (coordinates.size() == 2) {
        coordinates.push_back(coordinates[0]);
        coordinates.push_back(coordinates[1]);
        osm_node_ids.push_back(-1);
      }

      // TODO handle empty segments -> invent something based on station id

      fbs_segments.emplace_back(CreateSegment(mc, mc.CreateVector(coordinates),
                                              mc.CreateVector(osm_node_ids)));
    }

    return CreatePathSeqResponse(
        mc, mc.CreateVector(station_ids), mc.CreateVector(classes),
        mc.CreateVector(fbs_segments),
        mc.CreateVector(std::vector<Offset<PathSourceInfo>>{}));
  }

  std::unique_ptr<path_database> db_;
  std::unique_ptr<path_index> index_;

  tiles::render_ctx render_ctx_;
};

path::path() : module("Path", "path"), data_{std::make_unique<path::data>()} {
  param(database_path_, "db", "/path/to/pathdb.mdb");
}

path::~path() = default;

void path::init(registry& r) {
  try {
    data_->db_ = make_path_database(database_path_, true, false);

    data_->render_ctx_ = tiles::make_render_ctx(*data_->db_->db_handle_);

    if (auto buf = data_->db_->try_get(kIndexKey)) {
      data_->index_ = std::make_unique<path_index>(*buf);
    } else {
      LOG(warn) << "pathdb not available: no index!";
    }
  } catch (std::system_error const&) {
    LOG(warn) << "pathdb not available: no database!";
  }

  // used by: railviz
  r.register_op("/path/boxes", [this](msg_ptr const&) {
    verify_path_database_available();
    return boxes();
  });

  // used by: railviz, sim, legacydebugger
  r.register_op("/path/by_trip_id", [this](msg_ptr const& m) {
    verify_path_database_available();
    return by_trip_id(m);
  });

  // used by: sim, legacydebugger
  r.register_op("/path/by_station_seq", [this](msg_ptr const& m) {
    verify_path_database_available();
    return by_station_seq(m);
  });

  // used by: debugger
  r.register_op("/path/by_tile_feature", [this](msg_ptr const& m) {
    verify_path_database_available();
    return by_tile_feature(m);
  });

  // used by: debugger
  r.register_op("/path/tiles", [this](msg_ptr const& m) {
    verify_path_database_available();
    return path_tiles(m);
  });
}

void path::verify_path_database_available() const {
  if (!data_->db_ || !data_->index_) {
    throw std::system_error(error::database_not_available);
  }
}

msg_ptr path::boxes() const {
  auto const boxes = data_->db_->get(kBoxesKey);
  return make_msg(boxes.data(), boxes.size());
};

msg_ptr path::by_station_seq(msg_ptr const& msg) const {
  auto req = motis_content(PathByStationSeqRequest, msg);

  return data_->get_response(
      data_->index_->find({utl::to_vec(*req->station_ids(),
                                       [](auto const& s) { return s->str(); }),
                           req->clasz()}),
      req->zoom_level());
}

msg_ptr path::by_trip_id(msg_ptr const& msg) const {
  auto const& req = motis_content(PathByTripIdRequest, msg);
  auto const& sched = get_schedule();
  auto const& trp = from_fbs(sched, req->trip_id());

  auto const seq = utl::to_vec(access::stops(trp), [&sched](auto const& stop) {
    return stop.get_station(sched).eva_nr_.str();
  });

  auto const s = sections(trp);
  auto const clasz_it =
      std::min_element(begin(s), end(s), [](auto const& lhs, auto const& rhs) {
        return lhs.fcon().clasz_ < rhs.fcon().clasz_;
      });
  utl::verify(clasz_it != end(s), "invalid trip");

  return data_->get_response(
      data_->index_->find({seq, (*clasz_it).fcon().clasz_}), req->zoom_level());
}

msg_ptr path::by_tile_feature(msg_ptr const& msg) const {
  auto const& req = motis_content(PathByTileFeatureRequest, msg);

  message_creator mc;
  std::vector<Offset<PathSeqResponse>> responses;
  for (auto const& [seq, seg] : data_->index_->tile_features_.at(req->ref())) {
    responses.emplace_back(
        data_->reconstruct_sequence(mc, std::to_string(seq)));
  }

  mc.create_and_finish(
      MsgContent_MultiPathSeqResponse,
      CreateMultiPathSeqResponse(mc, mc.CreateVector(responses)).Union());
  return make_msg(mc);
}

msg_ptr path::path_tiles(msg_ptr const& msg) const {
  auto tile = tiles::parse_tile_url(msg->get()->destination()->target()->str());
  if (!tile) {
    throw std::system_error(error::invalid_request);
  }

  tiles::null_perf_counter pc;
  auto rendered_tile =
      tiles::get_tile(*data_->db_->db_handle_, *data_->db_->pack_handle_,
                      data_->render_ctx_, *tile, pc);

  message_creator mc;
  std::vector<Offset<HTTPHeader>> headers;
  Offset<String> payload;
  if (rendered_tile) {
    headers.emplace_back(CreateHTTPHeader(
        mc, mc.CreateString("Content-Type"),
        mc.CreateString("application/vnd.mapbox-vector-tile")));
    headers.emplace_back(CreateHTTPHeader(
        mc, mc.CreateString("Content-Encoding"), mc.CreateString("deflate")));
    payload = mc.CreateString(rendered_tile->data(), rendered_tile->size());
  } else {
    payload = mc.CreateString("");
  }

  mc.create_and_finish(
      MsgContent_HTTPResponse,
      CreateHTTPResponse(mc, HTTPStatus_OK, mc.CreateVector(headers), payload)
          .Union());

  return make_msg(mc);
}

}  // namespace motis::path
