use eframe::egui;
use walkers::{HttpTiles, MercatorProjection, sources};

use crate::app::MapProjection;

// Give Mercator tile sources the same projection type as the application's map memory.
struct MercatorTileSource<S>(S);

impl<S> sources::TileSource for MercatorTileSource<S>
where
    S: sources::TileSource<Projection = MercatorProjection>,
{
    type Projection = MapProjection;

    fn tile_url(&self, tile_id: walkers::TileId) -> String {
        self.0.tile_url(tile_id)
    }

    fn attribution(&self) -> sources::Attribution {
        self.0.attribution()
    }

    fn projection(&self) -> Self::Projection {
        MapProjection::default()
    }

    fn tile_size(&self) -> u32 {
        self.0.tile_size()
    }

    fn max_zoom(&self) -> u8 {
        self.0.max_zoom()
    }
}

#[expect(dead_code)]
#[derive(Debug, Default, Clone, Copy)]
pub enum GoogleServer {
    #[default]
    A = 1,
    B = 2,
    C = 3,
}

pub struct GoogleSatelliteSource(pub GoogleServer);

impl walkers::sources::TileSource for GoogleSatelliteSource {
    type Projection = walkers::MercatorProjection;

    fn tile_url(&self, tile_id: walkers::TileId) -> String {
        format!(
            "https://mt{}.google.com/vt/lyrs=s&x={}&y={}&z={}",
            self.0 as u8, tile_id.x, tile_id.y, tile_id.zoom
        )
    }

    fn attribution(&self) -> sources::Attribution {
        sources::Attribution {
            text: "Google Map Data",
            url: "https://www.google.com/maps/",
            logo_light: None,
            logo_dark: None,
        }
    }

    fn projection(&self) -> Self::Projection {
        MercatorProjection
    }
}

pub fn get_tile_sources(ctx: &egui::Context) -> BackgroundTiles {
    BackgroundTiles {
        osm: HttpTiles::new(MercatorTileSource(sources::OpenStreetMap), ctx.clone()),
        otm: HttpTiles::new(
            MercatorTileSource(sources::OpenTopoMap(sources::OpenTopoServer::C)),
            ctx.clone(),
        ),
        satellite: HttpTiles::new(
            MercatorTileSource(GoogleSatelliteSource(GoogleServer::C)),
            ctx.clone(),
        ),
    }
}

pub struct BackgroundTiles {
    pub osm: HttpTiles<MapProjection>,
    pub otm: HttpTiles<MapProjection>,
    pub satellite: HttpTiles<MapProjection>,
}
